package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.MiscUtils;
import frc.robot.util.PoseCamera;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final List<PoseCamera> mCameraList;
  private final Supplier<Rotation2d> m_gyroRotation;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositions;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private List<PhotonPipelineResult> results;

  public Vision(
      List<PoseCamera> cameraList,
      Supplier<Rotation2d> gyroRotation,
      Supplier<SwerveModulePosition[]> swerveModulePositions) {
    mCameraList = cameraList;
    m_gyroRotation = gyroRotation;
    m_swerveModulePositions = swerveModulePositions;
    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kSwerveDriveKinematics,
            m_gyroRotation.get(),
            m_swerveModulePositions.get(),
            new Pose2d());
    Logger.recordOutput("Robot/Pose/Current", new Pose2d());
  }

  @Override
  public void periodic() {
    UpdatePose();
    UpdateTelemetry();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  private void UpdatePose() {
    m_poseEstimator.update(m_gyroRotation.get(), m_swerveModulePositions.get());
    for (PoseCamera camera : mCameraList) {
      results = camera.getCameraResults();
      if (results.size() > 0) {
        camera
            .getEstimatedRobotPose(results.get(0))
            .ifPresent(
                estimatedRobotPose -> {
                  Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
                  if (isPoseOnField(pose)) {
                    if (estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                      m_poseEstimator.addVisionMeasurement(
                          pose,
                          estimatedRobotPose.timestampSeconds,
                          VisionConstants.kVisionMultiTagStandardDeviations);
                    } else {
                      for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                        if (MiscUtils.isValueInRange(
                            target.getPoseAmbiguity(),
                            0.0,
                            VisionConstants.kVisionMaxPoseAmbiguity)) {
                          m_poseEstimator.addVisionMeasurement(
                              pose,
                              estimatedRobotPose.timestampSeconds,
                              VisionConstants.kVisionSingleTagStandardDeviations);
                          break;
                        }
                      }
                    }
                  }
                });
      }
    }
    ;
  }

  private boolean isPoseOnField(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    return (x >= 0.0 && x <= VisionConstants.kAprilTagFieldLayout.getFieldLength())
        && (y >= 0.0 && y <= VisionConstants.kAprilTagFieldLayout.getFieldWidth());
  }

  public boolean hasVisionTargets() {
    for (PoseCamera camera : mCameraList) {
      if (camera.hasTarget()) {
        return true;
      }
    }
    return false;
  }

  public void UpdateTelemetry() {
    Logger.recordOutput("Robot/Pose/Current", m_poseEstimator.getEstimatedPosition());
    // for (PoseCamera camera : mCameraList) {
    //   Logger.recordOutput("Vision/Pose/" + camera.getCameraName(),
    // camera.getCameraPoseEstimate());
    // }
  }
}
