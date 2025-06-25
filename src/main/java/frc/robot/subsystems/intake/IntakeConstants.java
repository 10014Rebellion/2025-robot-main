package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConstants {
  public static class Beambreak {
    public static int kFrontSensorDIOPort = 3;
    public static int kBackSensorDIOPort = 4;
  }

  public static class Indexer {
    public static int kIndexerID = 55;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static int kCurrentLimit = 60;

    public static double kIntakeVolts = 9.5;
    public static double kIntakeVoltsSlow = 2.5;
    public static double kOuttakeVolts = -6;

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
    public static double kIntakeSpeed = 12;

    public static int kRollerCurrentLimit = 80;

    public static final SparkMaxConfig kRollerConfig = new SparkMaxConfig();

    static {
      kRollerConfig.idleMode(kIdleMode).smartCurrentLimit(kRollerCurrentLimit).inverted(kInverted);
    }
  }

  public static class IntakePivot {
    public static int kPivotID = 53;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kInverted = false;
    public static boolean kEncoderInverted = true;

    public static int kCurrentLimit = 60;

    public static double kPositionConversionFactor = 360.0;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static double kEncoderOffsetRev = 0.3345553;

    public static final SparkMaxConfig kPivotConfig = new SparkMaxConfig();

    public static double kTolerance = 3;
    public static double kP = 0.2;
    public static double kD = 0.001;
    public static double kMaxVelocity = 300; // Theoretical max: 1555
    public static double kMaxAcceleration = 500; // Theoretical max: 16179 deg/s^2

    public static double kForwardSoftLimit = 90.0; // TO DO: CONFIGURE ME!
    public static double kReverseSoftLimit = 2.0; // TO DO: CONFIGURE ME!

    public static double kS = 0.0; // 0.34
    public static double kG = 0.19; // 0.3
    public static double kV = 2.4; // 0.43
    public static double kA = 0.08; // 0.06

    public enum Setpoints {
      STOWED(90.0),
      ALGAEINTAKE(70),
      INTAKING(3.0);
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
      kPivotConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .inverted(kEncoderInverted)
          .zeroOffset(kEncoderOffsetRev);
    }
  }
}
