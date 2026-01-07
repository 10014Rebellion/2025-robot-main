package frc.robot.subsystems.vision.ObjDetection;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjDetection extends SubsystemBase{
    private ObjDetectionIO[] cameras;
    private ObjDetectionIOInputsAutoLogged[] camerasData;

    public static final AprilTagFieldLayout k2025Field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public ObjDetection(ObjDetectionIO[] cameras) {
        this.cameras = cameras;
        camerasData = new ObjDetectionIOInputsAutoLogged[cameras.length];
        for(int i = 0; i < cameras.length; i++) {
            camerasData[i] = new ObjDetectionIOInputsAutoLogged();
        }
    }

    public void periodic(Pose2d latestPose) {
        for(int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(camerasData[i], latestPose);
            LoggedMechanism2d coral = new LoggedMechanism2d(2000, 2000);
            if(camerasData[i].trackedTargetsCornersX.length != 0 && camerasData[i].trackedTargetsCornersY.length != 0) {
                LoggedMechanismRoot2d rootCoral = coral.getRoot(
                    "Coral0", 
                    camerasData[i].trackedTargetsCornersX[0], 
                    camerasData[i].trackedTargetsCornersY[0]);
            
                for(int j = 1; j < 4; j++) {
                    double mag = Math.hypot(
                        camerasData[i].trackedTargetsCornersX[j]
                        -
                        camerasData[i].trackedTargetsCornersX[j-1] , 
                        camerasData[i].trackedTargetsCornersY[j]
                        -
                        camerasData[i].trackedTargetsCornersY[j-1]);

                    double angleRad = Math.atan2
                    (camerasData[i].trackedTargetsCornersY[j]
                    -
                    camerasData[i].trackedTargetsCornersY[j-1] , 
                    camerasData[i].trackedTargetsCornersX[j]
                    -
                    camerasData[i].trackedTargetsCornersX[j-1]);

                    rootCoral.append(
                        new LoggedMechanismLigament2d("Coral"+j, mag, angleRad));
                }
            }

            Logger.recordOutput(camerasData[i].camName+i+"/coralllll", coral);

            Logger.processInputs("Vision/ObjDetection/"+camerasData[i].camName, camerasData[i]);
        }
    }

    private Transform2d toTransform2d(Transform3d transform) {
        return new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
    }

    // public void logVisionObservation(VisionObservation observation, String state) {
    //     Logger.recordOutput("Vision/Observation/"+observation.camName+"/State", state);
    //     Logger.recordOutput("Vision/Observation/"+observation.camName+"/Timestamp", observation.camName());
    //     Logger.recordOutput("Vision/Observation/"+observation.camName+"/Pose", observation.pose());
    //     Logger.recordOutput("Vision/Observation/"+observation.camName+"/hasObserved", observation.hasObserved());
    //     Logger.recordOutput("Vision/Observation/"+observation.camName+"/StdDevs", observation.stdDevs());
    // }
}