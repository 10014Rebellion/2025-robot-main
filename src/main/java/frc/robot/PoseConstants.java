package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Birds eye view of the field, assume that the processor side is south
public class PoseConstants {
  public static class Field {
    public static final double kFieldLengthM = 17.548;
    public static final double kFieldWidthM = 8.052;

    public static class Blue {
      public static final Pose2d kNorthLolipop = new Pose2d(1.2192, 4.026, Rotation2d.kZero);
      public static final Pose2d kMiddleLolipop = new Pose2d(1.2192, 4.026, Rotation2d.kZero);
      public static final Pose2d kSouthLolipop = new Pose2d(1.2192, 4.026, Rotation2d.kZero);
    }

    public static class Red {
      public static final Pose2d kNorthLolipop = new Pose2d(16.3288, 4.026, Rotation2d.k180deg);
      public static final Pose2d kMiddleLolipop = new Pose2d(16.3288, 4.026, Rotation2d.k180deg);
      public static final Pose2d kSouthLolipop = new Pose2d(16.3288, 4.026, Rotation2d.k180deg);
    }
  }
}
