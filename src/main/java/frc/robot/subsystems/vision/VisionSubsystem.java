package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionConstants.PoseOffsets;
import frc.robot.subsystems.vision.VisionConstants.linearPoseOffsets;
import frc.robot.util.MiscUtils;
import frc.robot.util.PoseCamera;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  private List<PoseCamera> mCameraList;
  private List<PhotonPipelineResult> results;
  private final DriveSubsystem mDriveSubsystem;

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

  public VisionSubsystem(
      DriveSubsystem pDriveSubsystem,
      Supplier<Rotation2d> gyroRotation,
      Supplier<SwerveModulePosition[]> swerveModulePositions) {
    initCameraList();
    this.mDriveSubsystem = pDriveSubsystem;
  }

  public int getClosestReefTag(boolean isBlueAlliance) { // , double pDistanceMeters) {
    // Determine valid tag IDs based on alliance
    int[] validTags =
        isBlueAlliance ? new int[] {17, 18, 19, 20, 21, 22} : new int[] {6, 7, 8, 9, 10, 11};
    Pose2d robotPose = mDriveSubsystem.getPose();
    Pose2d closestTagPose = null;
    double closestDistance = Double.MAX_VALUE;
    int closestTagId = -1;
    // Iterate through reef tags and find the closest one
    for (int tagId : validTags) {
      Pose2d tagPose =
          VisionConstants.kAprilTagFieldLayout.getTagPose(tagId).map(Pose3d::toPose2d).orElse(null);
      if (tagPose != null) {
        double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
        if (distance < closestDistance) {
          closestDistance = distance;
          closestTagPose = tagPose;
          closestTagId = tagId;
        }
      }
    }
    // If no valid tags found, return current pose
    if (closestTagPose == null) {
      return 0;
    }
    // Get the corrected pose in front of the closest reef tag
    return closestTagId;
  }

  @Override
  public void periodic() {
    updatePose();
    updateTelemetry();
  }

  public Pose2d getReefScoringPose(
      int pTagID, double pDistanceInches, Supplier<PoseOffsets> pOffset) {
    System.out.println("Pose offset left: " + pOffset.get().getOffsetM());
    return getPoseInFrontOfAprilTag(
        pTagID, Units.inchesToMeters(pDistanceInches), pOffset.get().getOffsetM());
  }

  public Pose2d getClosestReefScoringPose(
      Supplier<linearPoseOffsets> pDistanceOffset, Supplier<PoseOffsets> pOffset) {
    return getPoseInFrontOfAprilTag(
        getClosestReefTag(DriverStation.getAlliance().get().equals(Alliance.Blue)),
        pDistanceOffset.get().getOffsetM(),
        pOffset.get().getOffsetM());
  }

  public Pose2d getClosestReefScoringPose(PoseOffsets pOffset) {
    return getPoseInFrontOfAprilTag(
        getClosestReefTag(DriverStation.getAlliance().get().equals(Alliance.Blue)),
        VisionConstants.kScoringDistance,
        pOffset.getOffsetM());
  }

  public Pose2d getPoseInFrontOfAprilTag(int pTagID, PoseOffsets pOffset) {
    return getPoseInFrontOfAprilTag(pTagID, VisionConstants.kScoringDistance, pOffset.getOffsetM());
  }

  public Pose2d getPoseInFrontOfAprilTag(int pTagID, double pDistanceInches) {
    return getPoseInFrontOfAprilTag(pTagID, Units.inchesToMeters(pDistanceInches), 0);
  }

  public Pose2d getBranchPose(int branch) {
    return getPoseInFrontOfAprilTag(
        getBranchTag(branch), (branch % 2 == 0) ? PoseOffsets.RIGHT : PoseOffsets.LEFT);
  }

  public int getBranchTag(int branch) {
    switch (branch) {
      case 2:
      case 3:
        return 22;
      case 4:
      case 5:
        return 17;
      case 6:
      case 7:
        return 18;
      case 8:
      case 9:
        return 19;
      case 10:
      case 11:
        return 20;
      default:
        return 21;
    }
  }

  public Pose2d getPoseInFrontOfAprilTag(int pTagID, double pXOffsetM, double pYOffsetM) {
    Pose2d tagPose =
        VisionConstants.kAprilTagFieldLayout.getTagPose(pTagID).map(Pose3d::toPose2d).orElse(null);

    if (tagPose == null) {
      updatePose();
      return mDriveSubsystem.getPose();
    }

    // Calculate position in front of the tag
    Translation2d tagTranslation =
        tagPose
            .getTranslation()
            .plus(
                new Translation2d(
                        (VisionConstants.kRobotSideLength
                            / 2.0), // Offset the robot length so the front is 0 meters away
                        0)
                    .rotateBy(tagPose.getRotation()));
    Rotation2d tagRotation =
        tagPose.getRotation().plus(new Rotation2d(Math.PI)); // Flip the robot from the tag

    // Move "pDistanceInches" in the direction the tag is facing
    Translation2d frontOfTag =
        tagTranslation.plus(new Translation2d(-pXOffsetM, pYOffsetM).rotateBy(tagRotation));

    Pose2d targetPose2d =
        new Pose2d(frontOfTag, tagRotation)
            .plus(
                new Transform2d(
                    0,
                    0,
                    new Rotation2d(
                        Math.toRadians(0.0)))); // .plus(new Rotation2d(Math.toRadians(-90.0)));
    Logger.recordOutput("Robot/Vision/AprilTagSetpoint", targetPose2d);
    return targetPose2d;
  }

  private void updatePose() {
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
                      mDriveSubsystem.addVisionMeasurement(
                          pose,
                          estimatedRobotPose.timestampSeconds,
                          VisionConstants.kVisionMultiTagStandardDeviations);
                    } else {
                      for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                        if (MiscUtils.isValueInRange(
                            target.getPoseAmbiguity(),
                            0.0,
                            VisionConstants.kVisionMaxPoseAmbiguity)) {
                          mDriveSubsystem.addVisionMeasurement(
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

  public static void readPose(String label, Pose2d score) {
    SmartDashboard.putString(
        label,
        "x: " + score.getX() + ", y:" + score.getY() + ", deg:" + score.getRotation().getDegrees());
  }

  public void updateTelemetry() {
    readPose("C1", getBranchPose(1));
    readPose("C2", getBranchPose(2));
    readPose("C3", getBranchPose(3));
    readPose("C4", getBranchPose(4));
    readPose("C5", getBranchPose(5));
    readPose("C6", getBranchPose(6));
    readPose("C7", getBranchPose(7));
    readPose("C8", getBranchPose(8));
    readPose("C9", getBranchPose(9));
    readPose("C10", getBranchPose(10));
    readPose("C11", getBranchPose(11));
    readPose("C12", getBranchPose(12));

    boolean isConnected =
        false; // = mCameraList.get(0).isConnected() && mCameraList.get(1).isConnected();
    Logger.recordOutput("Robot/Vision/Cams Connected", isConnected);
    Logger.recordOutput("Robot/Vision/EstimatedPose", mDriveSubsystem.getPose());
    // for (PoseCamera camera : mCameraList) {
    //   Logger.recordOutput(
    //       "Vision/Pose/" + camera.getCameraName(), camera.getCameraPoseEstimate().toPose2d());
    // }
  }
}
