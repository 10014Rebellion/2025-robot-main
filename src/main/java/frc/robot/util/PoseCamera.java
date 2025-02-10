package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PoseCamera {
  private final PhotonCamera m_photonCamera;
  private final PhotonPoseEstimator m_photonPoseEstimator;

  public PoseCamera(
      String cameraName,
      Transform3d cameraTransform,
      PoseStrategy poseStrategy,
      PoseStrategy fallbackPoseStrategy,
      AprilTagFieldLayout aprilTagFieldLayout) {
    m_photonCamera = new PhotonCamera(cameraName);
    m_photonCamera.setDriverMode(false);
    m_photonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, cameraTransform);
    m_photonPoseEstimator.setMultiTagFallbackStrategy(fallbackPoseStrategy);
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose(PhotonPipelineResult result) {
    return m_photonPoseEstimator.update(result);
  }

  public List<PhotonPipelineResult> getCameraResults() {
    return m_photonCamera.getAllUnreadResults();
  }

  public Pose3d getCameraPoseEstimate() {
    if (m_photonCamera.getAllUnreadResults().size() > 0) {
      return m_photonPoseEstimator
          .update(m_photonCamera.getAllUnreadResults().get(0))
          .get()
          .estimatedPose;
    } else return new Pose3d();
  }

  public String getCameraName() {
    return m_photonCamera.getName();
  }

  public boolean hasTarget() {
    return m_photonCamera.getLatestResult().hasTargets();
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean(
        "Vision/Pose/" + m_photonCamera.getName() + "/IsConnected", m_photonCamera.isConnected());
    SmartDashboard.putBoolean(
        "Visions/Pose/" + m_photonCamera.getName() + "/HasTarget",
        m_photonCamera.getLatestResult().hasTargets());
  }
}
