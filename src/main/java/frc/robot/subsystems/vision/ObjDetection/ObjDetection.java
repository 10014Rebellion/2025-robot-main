package frc.robot.subsystems.vision.ObjDetection;

import static frc.robot.subsystems.vision.AprilTagDetection.AprilTagVisionConstants.KUseSingleTagTransform;
import static frc.robot.subsystems.vision.AprilTagDetection.AprilTagVisionConstants.kAmbiguityThreshold;
import static frc.robot.subsystems.vision.AprilTagDetection.AprilTagVisionConstants.kMultiStdDevs;
import static frc.robot.subsystems.vision.AprilTagDetection.AprilTagVisionConstants.kSingleStdDevs;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.ObjDetection.ObjDetectionIO.ObjDetectionIOInputs;
import frc.robot.util.debugging.LoggedTunableNumber;;

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

    @Override
    public void periodic() {
        for(int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(camerasData[i], new Pose2d(), new Pose2d());
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