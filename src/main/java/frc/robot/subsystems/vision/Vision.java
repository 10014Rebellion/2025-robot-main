package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

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

  public Pose2d getPoseInFrontOfAprilTag(int pTagID, double pDistanceMeters) {
    Pose2d tagPose =
        VisionConstants.kAprilTagFieldLayout.getTagPose(pTagID).map(Pose3d::toPose2d).orElse(null);

    if (tagPose == null) {
      updatePose();
      return mPoseEstimator.getEstimatedPosition();
    }

    // Calculate position in front of the tag
    Translation2d tagTranslation = tagPose.getTranslation();
    Rotation2d tagRotation = tagPose.getRotation();

    // Move "pDistanceMeters" in the direction the tag is facing
    Translation2d frontOfTag =
        tagTranslation.plus(new Translation2d(-pDistanceMeters, 0).rotateBy(tagRotation));

    return new Pose2d(frontOfTag, tagRotation);
  }

  public Pose2d getPoseInFrontOfClosestTag(double pDistanceMeters) {
    Pose2d tagPose = getClosestReefTag();
    if (tagPose == null || tagPose.equals(getPose())) {
      updatePose();
      return mPoseEstimator.getEstimatedPosition();
    }

    // Calculate position in front of the tag
    Translation2d tagTranslation = tagPose.getTranslation();
    Rotation2d tagRotation = tagPose.getRotation();

    // Move "pDistanceMeters" in the direction the tag is facing
    Translation2d frontOfTag = tagTranslation.plus(new Translation2d(pDistanceMeters, 0).rotateBy(tagRotation));

    return new Pose2d(frontOfTag, tagRotation);
  }

  private Pose2d getClosestReefTag() {
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    // Check if we have an alliance
    if (currentAlliance.isPresent()) {
      int closestTag = 0;
      // If alliance is blue, we're gonna loop through and find the closest blue reef tag
      if (currentAlliance.get() == Alliance.Blue) {

        // Set closest to be a really high value
        double closestDist = 10000;

        // Blue reef tags are from ID 17 to ID 22
        for (int ID = 17; ID <= 22; ID++) {

          // Check the distance between our current position and the tag position
          Pose2d tagPose = VisionConstants.kAprilTagFieldLayout.getTagPose(ID).map(Pose3d::toPose2d).orElse(null);
          double currentDist = tagPose.getTranslation().getDistance(getPose().getTranslation());

          if (currentDist < closestDist) {
            // If the current distance is less than closest, set closest tag to that tag 
            closestDist = currentDist;
            closestTag = ID;
          }
        }

        SmartDashboard.putNumber("Vision/Closest Tag ID", closestTag);

        // If we (somehow) dont find anything, ignore the values
        if (closestTag == 0) {
          return getPose();
        }

        return VisionConstants.kAprilTagFieldLayout.getTagPose(closestTag).map(Pose3d::toPose2d).orElse(getPose());
      }

      // If alliance is red, we're gonna loop through and find the closest red reef tag
      if (currentAlliance.get() == Alliance.Red) {

        // Set closest to be a really high value
        double closestDist = 10000;

        // Red reef tags are from ID 6 to ID 11
        for (int ID = 6; ID <= 11; ID++) {

          // Check the distance between our current position and the tag position
          Pose2d tagPose = VisionConstants.kAprilTagFieldLayout.getTagPose(ID).map(Pose3d::toPose2d).orElse(null);
          double currentDist = tagPose.getTranslation().getDistance(getPose().getTranslation());

          if (currentDist < closestDist) {
            // If the current distance is less than closest, set closest tag to that tag 
            closestDist = currentDist;
            closestTag = ID;
          }
        }
        
        SmartDashboard.putNumber("Vision/Closest Tag ID", closestTag);
        // If we (somehow) dont find anything, ignore the values
        if (closestTag == 0) {
          return getPose();
        }
        return VisionConstants.kAprilTagFieldLayout.getTagPose(closestTag).map(Pose3d::toPose2d).orElse(getPose());
      }
    }

    return getPose();
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
