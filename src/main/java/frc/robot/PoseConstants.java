package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

// Birds eye view of the field, assume that the processor side is south, collect poses from blue
// side
public class PoseConstants {
  public static class WeldedField {
    public static final double kFieldLengthM = 17.548;
    public static final double kFieldWidthM = 8.052;
    public static final double kPipeDistanceM = 0.3302;
    public static final double kStartingLineX = 7.6057252;
    public static final double kBargeCenterX = 8.774;
    public static final double kDistAwayFromBargeX = Units.inchesToMeters(65);

    public static final double kBargeAlignBlueX = kBargeCenterX - kDistAwayFromBargeX; // Blue inner
    public static final double kBargeAlignRedX = kBargeCenterX + kDistAwayFromBargeX; // Red Inner

    public static final double kBargeAlignEastDeg = 180;
    public static final double kBargeAlignWestDeg = 0;

    public static final Pose2d kBargeScorePoseBlue =
        new Pose2d(7.31, 0.0, new Rotation2d(Units.degreesToRadians(2.7)));
  }
}
