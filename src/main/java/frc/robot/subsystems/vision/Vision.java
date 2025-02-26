package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.MiscUtils;
import frc.robot.util.PoseCamera;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private List<PoseCamera> mCameraList;
  private final Supplier<Rotation2d> mGyroRotation;
  private final Supplier<SwerveModulePosition[]> mSwerveModulePositions;
  private final SwerveDrivePoseEstimator mPoseEstimator;
  private List<PhotonPipelineResult> results;

  private void initCameraList() {
    List<PoseCamera> poseCameras = new ArrayList<>();

    String[] cameraNames =
        new String[] {
          VisionConstants.FRONT_LEFT_CAM,
          VisionConstants.FRONT_RIGHT_CAM,
          VisionConstants.BACK_LEFT_CAM,
          VisionConstants.BACK_RIGHT_CAM
        };

    for (String name : cameraNames) {
      poseCameras.add(
          new PoseCamera(
              name,
              VisionConstants.cameraPositions.get(name),
              VisionConstants.kPoseStrategy,
              VisionConstants.kFallbackPoseStrategy,
              VisionConstants.kAprilTagFieldLayout));
    }

    mCameraList = poseCameras;
  }

  public Vision(
      Supplier<Rotation2d> gyroRotation, Supplier<SwerveModulePosition[]> swerveModulePositions) {

    initCameraList();
    mGyroRotation = gyroRotation;
    mSwerveModulePositions = swerveModulePositions;
    mPoseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kSwerveDriveKinematics,
            mGyroRotation.get(),
            mSwerveModulePositions.get(),
            new Pose2d());

    Logger.recordOutput("Robot/Pose/Current", new Pose2d());
  }

  @Override
  public void periodic() {
    updatePose();
    updateTelemetry();
  }

  public Pose2d getPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  private void updatePose() {
    mPoseEstimator.update(mGyroRotation.get(), mSwerveModulePositions.get());
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
                      mPoseEstimator.addVisionMeasurement(
                          pose,
                          estimatedRobotPose.timestampSeconds,
                          VisionConstants.kVisionMultiTagStandardDeviations);
                    } else {
                      for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                        if (MiscUtils.isValueInRange(
                            target.getPoseAmbiguity(),
                            0.0,
                            VisionConstants.kVisionMaxPoseAmbiguity)) {
                          mPoseEstimator.addVisionMeasurement(
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

  public void updateTelemetry() {
    Logger.recordOutput("Robot/Pose/Current", mPoseEstimator.getEstimatedPosition());
    // for (PoseCamera camera : mCameraList) {
    //   Logger.recordOutput("Vision/Pose/" + camera.getCameraName(),
    // camera.getCameraPoseEstimate());
    // }
  }
}
