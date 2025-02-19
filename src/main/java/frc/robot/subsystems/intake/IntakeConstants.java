package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConstants {
  public static class Funnel {
    public static int kMotorID = 55;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static int kCurrentLimit = 40;

    public static final SparkMaxConfig kFunnelConfig = new SparkMaxConfig();

    static {
      kFunnelConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);
    }
  }

  public static class OTBIntake {
    public static int kLeftPivotID = 51;
    public static int kRightPivotID = 53;
    public static int kLeftRollerID = 52;
    public static int kRightRollerID = 54;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kPivotIdleMode = IdleMode.kBrake;
    public static IdleMode kRollerIdleMode = IdleMode.kCoast;
    public static int kRollerCurrentLimit = 40;
    public static int kPivotCurrentLimit = 40;
    public static double kPositionConversionFactor = 360;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60;
    public static double kLeftEncoderOffsetRev = 0;
    public static double kRightEncoderOffsetRev = 0;

    public static final SparkMaxConfig kLeftPivotConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightPivotConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRollerConfig = new SparkMaxConfig();

    static {
      // kLeftPivotConfig.idleMode(kPivotIdleMode).smartCurrentLimit(kCurrentLimit);
      kRightPivotConfig.idleMode(kPivotIdleMode).smartCurrentLimit(kPivotCurrentLimit);
      kRollerConfig.idleMode(kRollerIdleMode).smartCurrentLimit(kRollerCurrentLimit);
      // kLeftPivotConfig
      //     .absoluteEncoder
      //     .positionConversionFactor(kPositionConversionFactor)
      //     .velocityConversionFactor(kVelocityConversionFactor)
      //     .zeroOffset(kLeftEncoderOffsetRev);
      kRightPivotConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .zeroOffset(kRightEncoderOffsetRev);
    }
  }
}
