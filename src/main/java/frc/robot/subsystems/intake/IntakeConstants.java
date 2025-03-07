package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConstants {
  public static class Funnel {
    public static int kFunnelID = 55;
    public static int kIndexerID = 58;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static int kCurrentLimit = 60;

    public static final SparkMaxConfig kFunnelConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kIndexerConfig = new SparkMaxConfig();

    static {
      kFunnelConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(true);
      kIndexerConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(true);
    }
  }

  public static class OTBIntakeConstants {
    public static int kLeftPivotID = 51;
    public static int kRightPivotID = 53;
    public static int kLeftRollerID = 52;
    public static int kRightRollerID = 54;
    public static int kEncoderDIOPort = 0;
    public static int kSensor1DIOPort = 2;
    public static int kSensor2DIOPort = 3;
    public static double kEncoderOffset = 130;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kPivotIdleMode = IdleMode.kBrake;
    public static IdleMode kRollerIdleMode = IdleMode.kCoast;
    public static int kRollerCurrentLimit = 60;
    public static int kPivotCurrentLimit = 60;

    public static double kPositionConversionFactor = 360;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static double kLeftEncoderOffsetRev = 0;
    public static double kRightEncoderOffsetRev = 0;

    public static final SparkMaxConfig kLeftPivotConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightPivotConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRollerConfig = new SparkMaxConfig();

    public static double kTolerance = 1;
    public static double kP = 0.08;
    public static double kD = 0;
    public static double kForwardSoftLimit = 100.0; // TO DO: CONFIGURE ME!
    public static double kReverseSoftLimit = 0.0; // TO DO: CONFIGURE ME!

    public static double kG = 0.1;

    static {
      // kLeftPivotConfig.idleMode(kPivotIdleMode).smartCurrentLimit(kCurrentLimit);
      kRightPivotConfig
          .idleMode(kPivotIdleMode)
          .smartCurrentLimit(kPivotCurrentLimit)
          .inverted(false);
      kRollerConfig.idleMode(kRollerIdleMode).smartCurrentLimit(kRollerCurrentLimit);
      // kLeftPivotConfig
      //     .absoluteEncoder
      //     .positionConversionFactor(kPositionConversionFactor)
      //     .velocityConversionFactor(kVelocityConversionFactor)
      //     .zeroOffset(kLeftEncoderOffsetRev);
    }
  }

  public enum IntakePositions {
    STOWED(100.0),
    INTAKING(0.0);
    public final double position;

    private IntakePositions(double position) {
      this.position = position;
    }

    public double getPos() {
      return this.position;
    }
  };
}
