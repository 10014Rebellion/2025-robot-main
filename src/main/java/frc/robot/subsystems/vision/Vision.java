package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants.PoseOffsets;
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
  }

  @Override
  public void periodic() {
    updatePose();
    updateTelemetry();
  }

  public Pose2d getReefScoringPose(int pTagID, double pDistanceInches, PoseOffsets pOffset) {
    return getPoseInFrontOfAprilTag(
        pTagID, Units.inchesToMeters(pDistanceInches), pOffset.getOffsetM());
  }

  public Pose2d getPoseInFrontOfAprilTag(int pTagID, double pDistanceInches) {
    return getPoseInFrontOfAprilTag(pTagID, Units.inchesToMeters(pDistanceInches), 0);
  }

  public Pose2d getPoseInFrontOfAprilTag(int pTagID, double pXOffsetM, double pYOffsetM) {
    Pose2d tagPose =
        VisionConstants.kAprilTagFieldLayout.getTagPose(pTagID).map(Pose3d::toPose2d).orElse(null);

    if (tagPose == null) {
      updatePose();
      return mPoseEstimator.getEstimatedPosition();
    }

    // Calculate position in front of the tag
    Translation2d tagTranslation =
        tagPose
            .getTranslation()
            .plus(
                new Translation2d(
                    (VisionConstants.kRobotYLength
                        / 2.0), // Offset the robot length so the front is 0 meters away
                    0));
    Rotation2d tagRotation =
        tagPose.getRotation().plus(new Rotation2d(Math.PI)); // Flip the robot from the tag

    // Move "pDistanceInches" in the direction the tag is facing
    Translation2d frontOfTag =
        tagTranslation.plus(new Translation2d(-pXOffsetM, pYOffsetM).rotateBy(tagRotation));

    Pose2d targetPose2d = new Pose2d(frontOfTag, tagRotation);
    Logger.recordOutput("Robot/Vision/AprilTagSetpoint", targetPose2d);
    return targetPose2d;
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
    Logger.recordOutput("Robot/Vision/EstimatedPose", mPoseEstimator.getEstimatedPosition());
    // for (PoseCamera camera : mCameraList) {
    //   Logger.recordOutput("Vision/Pose/" + camera.getCameraName(),
    // camera.getCameraPoseEstimate());
    // }
  }
}
