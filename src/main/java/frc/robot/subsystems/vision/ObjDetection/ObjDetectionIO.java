package frc.robot.subsystems.vision.ObjDetection;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

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
        public double bestPoseAmbiguity = 0.0;
        public String bestTargetClass = "";
        public double latencySeconds = 0.0;

        public boolean hasTarget = false;
        public int numberOfTargets = 0;

        public boolean hasBeenUpdated = false;

        public Transform3d cameraToRobot = new Transform3d();
        public Transform3d cameraToObj = new Transform3d();
        public Transform3d robotToObj = new Transform3d(); 

        public Pose3d[] trackedTargetsPose = new Pose3d[] {};
        public String[] trackedTargetsClass = new String[] {};
        
        public PhotonPipelineResult result = new PhotonPipelineResult();

    }

    public default void updateInputs(ObjDetectionIOInputs inputs, Pose2d latestPose) {}
    
}
