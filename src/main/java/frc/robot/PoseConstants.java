package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.PoseUtils;

// Birds eye view of the field, assume that the processor side is south
public class PoseConstants {
  public static class AndyMarkField {
    public static final double kFieldLengthM = 17.548;
    public static final double kFieldWidthM = 8.042;
    public static final double kPipeDistanceM = 0.3302;
    public static final double kStartingLineX =
        7.6057252; // Measured from the inside of starting line

    public static class Blue {
      public static final Pose2d kNorthLolipop = new Pose2d(1.2192, 5.8548, Rotation2d.k180deg);
      public static final Pose2d kMiddleLolipop = new Pose2d(1.2192, 4.026, Rotation2d.k180deg);
      public static final Pose2d kSouthLolipop = new Pose2d(1.2192, 2.1972, Rotation2d.k180deg);

      public static final Pose2d kTag17Pose =
          new Pose2d(4.073905999999999, 3.3012379999999997, Rotation2d.fromDegrees(60));
      public static final Pose2d kTag18Pose =
          new Pose2d(3.6576, 4.0208200000000005, Rotation2d.fromDegrees(0));
      public static final Pose2d kTag19Pose =
          new Pose2d(4.073905999999999, 4.740402, Rotation2d.fromDegrees(-60));
      public static final Pose2d kTag20Pose =
          new Pose2d(4.904739999999999, 4.740402, Rotation2d.fromDegrees(240));
      public static final Pose2d kTag21Pose =
          new Pose2d(5.321046, 4.0208200000000005, Rotation2d.fromDegrees(180));
      public static final Pose2d kTag22Pose =
          new Pose2d(4.904739999999999, 3.3012379999999997, Rotation2d.fromDegrees(-240));

      public static final Pose2d kNorthCage = new Pose2d(8.7738712, 7.2841866, Rotation2d.kZero);
      public static final Pose2d kMiddleCage = new Pose2d(8.7738712, 6.168517, Rotation2d.kZero);
      public static final Pose2d kSouthCage = new Pose2d(8.7738712, 5.0786538, Rotation2d.kZero);

      public static final Pose2d kNorthStartOuter =
          new Pose2d(7.6057252, 7.2841866, Rotation2d.k180deg);
      public static final Pose2d kNorthStartCenter =
          new Pose2d(7.6057252, 6.168517, Rotation2d.k180deg);
      public static final Pose2d kNorthStartInner =
          new Pose2d(7.6057252, 5.0786538, Rotation2d.k180deg);

      public static final Pose2d kSouthStartInner =
          new Pose2d(7.6057252, 2.9633462, Rotation2d.k180deg);
      public static final Pose2d kSouthStartCenter =
          new Pose2d(7.6057252, 1.873483, Rotation2d.k180deg);
      public static final Pose2d kSouthStartOuter =
          new Pose2d(7.6057252, 0.7578134, Rotation2d.k180deg);
    }

    public static class Red {
      public static final Pose2d kNorthLolipop = new Pose2d(16.3288, 5.8548, Rotation2d.kZero);
      public static final Pose2d kMiddleLolipop = new Pose2d(16.3288, 4.026, Rotation2d.kZero);
      public static final Pose2d kSouthLolipop = new Pose2d(16.3288, 2.1972, Rotation2d.kZero);
    }
  }

  public static class Bot {
    public static final double kBackupPose =
        -1 * ((VisionConstants.kRobotSideLength / 2.0) + VisionConstants.kScoringDistance);
    public static final Pose2d kC12Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag21Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.RIGHT.getOffsetM());
    public static final Pose2d kC1Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag21Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.LEFT.getOffsetM());

    public static final Pose2d kC2Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag22Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.RIGHT.getOffsetM());
    public static final Pose2d kC3Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag22Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.LEFT.getOffsetM());

    public static final Pose2d kC4Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag17Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.RIGHT.getOffsetM());
    public static final Pose2d kC5Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag17Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.LEFT.getOffsetM());

    public static final Pose2d kC6Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag18Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.RIGHT.getOffsetM());
    public static final Pose2d kC7Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag18Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.LEFT.getOffsetM());

    public static final Pose2d kC8Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag19Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.RIGHT.getOffsetM());
    public static final Pose2d kC9Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag19Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.LEFT.getOffsetM());

    public static final Pose2d kC10Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag20Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.RIGHT.getOffsetM());
    public static final Pose2d kC11Score =
        PoseUtils.shiftPose(
            AndyMarkField.Blue.kTag20Pose,
            kBackupPose,
            VisionConstants.PoseOffsets.LEFT.getOffsetM());
  }

  public static void readPose(String label, Pose2d score) {
    SmartDashboard.putString(
        label,
        "x: " + score.getX() + ", y:" + score.getY() + ", deg:" + score.getRotation().getDegrees());
  }
}
