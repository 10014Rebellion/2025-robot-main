package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Birds eye view of the field, assume that the processor side is south
public class PoseConstants {
  public static class AndyMarkField {
    public static final double kFieldLengthM = 17.548;
    public static final double kFieldWidthM = 8.042;
    public static final double kPipeDistanceM = 0.3302;

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
    }

    public static class Red {
      public static final Pose2d kNorthLolipop = new Pose2d(16.3288, 5.8548, Rotation2d.kZero);
      public static final Pose2d kMiddleLolipop = new Pose2d(16.3288, 4.026, Rotation2d.kZero);
      public static final Pose2d kSouthLolipop = new Pose2d(16.3288, 2.1972, Rotation2d.kZero);
    }
  }

  static {
  }
}
