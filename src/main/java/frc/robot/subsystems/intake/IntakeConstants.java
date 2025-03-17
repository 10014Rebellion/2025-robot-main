package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConstants {
  public static class Beambreak {
    public static int kFrontSensorDIOPort = 5;
    public static int kBackSensorDIOPort = 6;
  }

  public static class Indexer {
    public static int kIndexerID = 55;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static int kCurrentLimit = 60;

    public static final SparkMaxConfig kIndexerConfig = new SparkMaxConfig();

    static {
      kIndexerConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(true);
    }
  }

  public static class IntakeRoller {
    public static int kRollerID = 54;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static boolean kInverted = false;

    public static int kRollerCurrentLimit = 80;

    public static final SparkMaxConfig kRollerConfig = new SparkMaxConfig();

    static {
      kRollerConfig.idleMode(kIdleMode).smartCurrentLimit(kRollerCurrentLimit).inverted(kInverted);
    }
  }

  public static class IntakePivot {
    public static int kPivotID = 53;
    public static int kEncoderPort = 0;
    public static double kEncoderOffsetDeg = -105.0;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kInverted = false;

    public static int kCurrentLimit = 60;

    public static double kPositionConversionFactor = 360.0;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static double kEncoderOffsetRev = 0;

    public static final SparkMaxConfig kPivotConfig = new SparkMaxConfig();

    public static double kTolerance = 1;
    public static double kP = 0.08;
    public static double kD = 0;
    public static double kMaxVelocity = 0;
    public static double kMaxAcceleration = 0;

    public static double kForwardSoftLimit = 100.0; // TO DO: CONFIGURE ME!
    public static double kReverseSoftLimit = 0.0; // TO DO: CONFIGURE ME!

    public static double kS = 0.0;
    public static double kG = 0.1;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public enum Setpoints {
      STOWED(100.0),
      ALGAEINTAKE(75),
      INTAKING(0.0);
      public final double position;

      private Setpoints(double position) {
        this.position = position;
      }

      public double getPos() {
        return this.position;
      }
    };

    static {
      kPivotConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kInverted);
    }
  }
}
