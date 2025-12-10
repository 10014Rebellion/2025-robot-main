package frc.robot.subsystems.vision.ObjDetection;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.AprilTagDetection.AprilTagVisionConstants.Orientation;

public class ObjDetectionIOPV implements ObjDetectionIO{
    private String camName;
    private PhotonCamera photonCamera;
    private Transform3d cameraTransform;
    private Orientation orientation;

    public ObjDetectionIOPV(String name, Transform3d cameraTransform, Orientation orientation) {
        camName = name;
        photonCamera = new PhotonCamera(camName);
        this.orientation = orientation;
        this.cameraTransform = cameraTransform;

        PhotonCamera.setVersionCheckEnabled(false);
    }

    // Try and make an algorithm to prioritize coral that are isolated than ones that are bunched up //
    public PhotonPipelineResult findBestTarget(PhotonPipelineResult result){
        return new PhotonPipelineResult();
    }

    @Override
    public void updateInputs(ObjDetectionIOInputs inputs, Pose2d lastRobotPose, Pose2d simOdomPose){
        inputs.camName = camName;
        inputs.cameraToRobot = cameraTransform;

        try {

            if(Constants.currentMode == Mode.SIM){
            }

            List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
            inputs.hasBeenUpdated = !unreadResults.isEmpty();

            if(!unreadResults.isEmpty()){

                // Grabbing the very last result (most recent) //
                // Maybe just use -1 instead of unreadResults.size() -1 //
                PhotonPipelineResult result = unreadResults.get(unreadResults.size()-1);

                inputs.isConnected = photonCamera.isConnected();
                inputs.hasTarget = result.hasTargets();

                if(result.hasTargets()){
                    PhotonTrackedTarget target = result.getBestTarget();
                    inputs.bestTargetArea = target.area;
                    inputs.bestTargetPitch = target.pitch;
                    inputs.bestTargetYaw = target.yaw;
                    inputs.bestPoseAmbiguity = target.poseAmbiguity;

                    inputs.latencySeconds = result.getTimestampSeconds() / 1000.0;
                    inputs.numberOfTargets = result.targets.size();

                    inputs.cameraToObj = target.getBestCameraToTarget();
                    inputs.robotToObj = target.getBestCameraToTarget().plus(cameraTransform);

                    inputs.result = result;
                    

                }
            }
        }

        catch(Exception e) {
            inputs.camName = "";
            inputs.isConnected = false;
    
            inputs.bestTargetYaw = 0;
            inputs.bestTargetPitch = 0;
            inputs.bestTargetArea = 0;
            inputs.bestPoseAmbiguity = 0;
            inputs.latencySeconds = 0;
    
            inputs.hasTarget = false;
            inputs.numberOfTargets = 0;
    
            inputs.hasBeenUpdated = false;
    
            inputs.cameraToRobot = new Transform3d();
            inputs.cameraToObj = new Transform3d();
            inputs.robotToObj = new Transform3d(); 
            
            inputs.result = new PhotonPipelineResult();

        }
    }
    
}
