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

    public String mapClass(int label){
        return (label == 0) ? "Algae" : "Coral";
    }

    @Override
    public void updateInputs(ObjDetectionIOInputs inputs, Pose2d latestPose){

        inputs.camName = camName;

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

                    // The object detection model maps the objDetectId as 0 for algae and 1 for coral //
                    inputs.bestTargetClass = mapClass(target.objDetectId);

                    inputs.latencySeconds = result.getTimestampSeconds() / 1000.0;
                    inputs.numberOfTargets = result.targets.size();

                    String[] targetTypes = new String[result.targets.size()];


                    double[] areas = new double[result.targets.size()];
                    double[] pitches = new double[result.targets.size()];
                    double[] yaws = new double[result.targets.size()];
                    String[] classes = new String[result.targets.size()];

                    if(result.hasTargets()){
                        for(int i = 0; i < result.targets.size(); i++){
                            areas[i] = result.targets.get(i).area;
                            pitches[i] = result.targets.get(i).pitch;
                            yaws[i] = result.targets.get(i).yaw;
                            classes[i] = mapClass(result.targets.get(i).objDetectId);
                        }
                    }


                    inputs.trackedTargetsClass = targetTypes;

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
            inputs.bestTargetClass = "";
            inputs.latencySeconds = 0;
    
            inputs.hasTarget = false;
            inputs.numberOfTargets = 0;
    
            inputs.hasBeenUpdated = false;
            
            inputs.result = new PhotonPipelineResult();

        }
    }
    
}
