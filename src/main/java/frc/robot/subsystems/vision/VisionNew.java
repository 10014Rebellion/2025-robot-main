package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionNew extends SubsystemBase {
  PhotonCamera cam1;
  AprilTagFieldLayout AFL = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Transform3d robotToCam =
      new Transform3d(
          new Translation3d(0, 0, 0), // 0.37, .32, .29),
          new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))); // 0, -37, 22

  PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(AFL, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
  private final Field2d m_field = new Field2d();
  Pose3d pose = new Pose3d();

  public VisionNew() {
    cam1 = new PhotonCamera(VisionConstants.cam1Name);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult result) {
    return photonPoseEstimator.update(result);
  }

  @Override
  public void periodic() {
    int id = 0;
    var results = cam1.getAllUnreadResults();
    if (results.size() > 0) {
      PhotonPipelineResult result = results.get(0);
      if (result.hasTargets()) {
        var target = result.getBestTarget();
        System.out.println("Hi there im a value");
        pose = getEstimatedGlobalPose(result).get().estimatedPose;
      }
    }
    if (pose != null) {
      SmartDashboard.putData("Vision/Field", m_field);
      m_field.setRobotPose(AFL.getTagPose(5).get().toPose2d());
      SmartDashboard.putNumber("Vision/Pose", pose.getX());
      // SmartDashboard.putData("Vision/AP1", AFL.getTagPose(7).get().toPose2d());
    }
  }
}
