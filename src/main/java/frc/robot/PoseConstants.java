package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

// Birds eye view of the field, assume that the processor side is south
public class PoseConstants {
  public static class AndyMarkField {
    public static final double kFieldLengthM = 17.548;
    public static final double kFieldWidthM = 8.042;
    public static final double kPipeDistanceM = 0.3302;
    public static final double kStartingLineX =
        7.6057252; // Measured from the inside of starting line

    public static class Blue {
      public static final Pose2d kNorthLolipop = new Pose2d(1.2192, 5.8498, Rotation2d.k180deg);
      public static final Pose2d kMiddleLolipop = new Pose2d(1.2192, 4.021, Rotation2d.k180deg);
      public static final Pose2d kSouthLolipop = new Pose2d(1.2192, 2.1922, Rotation2d.k180deg);

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

  }

  public static class Bot {
    public static final Pose2d kC1Scoring = new Pose2d(5.759196, 3.8227, new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d kC2Scoring = new Pose2d(5.233800226280623, 2.990368969331848, new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d kC3Scoring = new Pose2d(4.947838637951001, 2.825268969331848, new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d kC4Scoring = new Pose2d(3.9648162262806226, 2.863368969331848, new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d kC5Scoring = new Pose2d(3.678854637951001, 3.0284689693318483, new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d kC6Scoring = new Pose2d(3.21945, 3.8989000000000003, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d kC7Scoring = new Pose2d(3.21945, 4.2291, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d kC8Scoring = new Pose2d(3.744845773719376, 5.061431030668151, new Rotation2d(Units.degreesToRadians(-60)));
    public static final Pose2d kC9Scoring = new Pose2d(4.030807362048997, 5.226531030668152, new Rotation2d(Units.degreesToRadians(-60)));
    public static final Pose2d kC10Scoring = new Pose2d(5.013829773719376, 5.188431030668152, new Rotation2d(Units.degreesToRadians(-120)));
    public static final Pose2d kC11Scoring = new Pose2d(5.299791362048998, 5.023331030668151, new Rotation2d(Units.degreesToRadians(-120)));
    public static final Pose2d kC12Scoring = new Pose2d(5.759196, 4.1529, new Rotation2d(Units.degreesToRadians(180)));
  }
}