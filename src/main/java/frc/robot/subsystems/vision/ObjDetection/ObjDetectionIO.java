package frc.robot.subsystems.vision.ObjDetection;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.targeting.proto.TargetCornerProto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface ObjDetectionIO {

    @AutoLog
    public static class ObjDetectionIOInputs {
        public String camName = "";
        public boolean isConnected = false;

        public double bestTargetYaw = 0.0;
        public double bestTargetPitch = 0.0;
        public double bestTargetArea = 0.0;
        public String bestTargetClass = "";
        public double latencySeconds = 0.0;

        public boolean hasTarget = false;
        public int numberOfTargets = 0;

        public boolean hasBeenUpdated = false;

        public String[] trackedTargetsClass = new String[] {};
        public double[] trackedTargetsArea = new double[] {};
        public double[] trackedTargetsPitch = new double[] {};
        public double[] trackedTargetsYaw = new double[] {};     

        public double[] trackedTargetsCornersX = new double[] {};
        public double[] trackedTargetsCornersY = new double[] {};
        
        public PhotonPipelineResult result = new PhotonPipelineResult();

    }

    public default void updateInputs(ObjDetectionIOInputs inputs, Pose2d latestPose) {}
    
}
