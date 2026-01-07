package frc.robot.subsystems.vision.ObjDetection;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.TargetCornerProto;

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

    public String findOrientation(double[] xs, double[] ys){
        

        return "toilet";
    }

    @Override
    public void updateInputs(ObjDetectionIOInputs inputs, Pose2d latestPose){

        inputs.camName = camName;

        try {

            if(Constants.currentMode == Mode.SIM){
            }

            List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
            System.out.println(unreadResults.size());
            inputs.hasBeenUpdated = unreadResults.size() != 0;


            if(!unreadResults.isEmpty()){

                // Grabbing the very last result (most recent) //
                // Maybe just use -1 instead of unreadResults.size() -1 //
                PhotonPipelineResult result = unreadResults.get(unreadResults.size() - 1);


                inputs.result = result;
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

                    List<Double> x = new ArrayList<Double>();
                    List<Double> y = new ArrayList<Double>();

                    for(int i = 0; i < result.targets.size(); i++){
                        areas[i] = result.targets.get(i).area;
                        pitches[i] = result.targets.get(i).pitch;
                        yaws[i] = result.targets.get(i).yaw;
                        classes[i] = mapClass(result.targets.get(i).objDetectId);

                        Logger.recordOutput(camName+"i"+"j"+"x", result.targets.get(i).getMinAreaRectCorners().get(0).x);
                        Logger.recordOutput(camName+"i"+"j"+"y", result.targets.get(i).getMinAreaRectCorners().get(0).y);
                        x.add(result.targets.get(i).getMinAreaRectCorners().get(0).x);
                        y.add(result.targets.get(i).getMinAreaRectCorners().get(0).y);

                        x.add(result.targets.get(i).getMinAreaRectCorners().get(1).x);
                        y.add(result.targets.get(i).getMinAreaRectCorners().get(1).y);

                        x.add(result.targets.get(i).getMinAreaRectCorners().get(2).x);
                        y.add(result.targets.get(i).getMinAreaRectCorners().get(2).y);

                        x.add(result.targets.get(i).getMinAreaRectCorners().get(3).x);
                        y.add(result.targets.get(i).getMinAreaRectCorners().get(3).y);
                    }

                    inputs.trackedTargetsArea = areas;
                    inputs.trackedTargetsClass = classes;
                    inputs.trackedTargetsPitch = pitches;
                    inputs.trackedTargetsYaw = yaws;
                    
                    double[] X = new double[x.size()];
                    double[] Y = new double[y.size()];
                    for(int i = 0; i < x.size(); i++) {
                        X[i]  = x.get(i);
                        Y[i]  = y.get(i);
                    }

                    inputs.trackedTargetsCornersX = X;
                    inputs.trackedTargetsCornersY = Y;
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
