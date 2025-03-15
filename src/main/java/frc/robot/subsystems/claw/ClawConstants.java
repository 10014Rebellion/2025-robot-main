package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawConstants {

  public static final int kClawID = 43;
  public static final int kEncoderDIOPort = 1;
  // public static final double kEncoderOffset = 48.5; // TODO: TUNE

  public static final MotorType kMotorType = MotorType.kBrushless;
  public static final IdleMode kIdleMode = IdleMode.kBrake;
  public static final int kCurrentLimit = 80;
  public static final boolean kInverted = false;
  public static final double kP = 0.01; // TODO: Configure me!
  public static final double kD = 0.0; // TODO: Configure me!

  public static final double kEncoderOffsetRev = 0.18528; // In revolutions

  public static final double kPositionConversionFactor = 360.0;
  public static final double kVelocityConversionFactor =
      kPositionConversionFactor / 60.0; // RPM -> MPS

  public static final SparkMaxConfig kClawConfig = new SparkMaxConfig();

  public enum Setpoints {
    INTAKE_CORAL(1),
    INTAKE_ALGAE(3),
    HOLD_ALGAE(0.5),
    OUTTAKE_REEF(-0.5),
    OUTTAKE_BARGE(-8),
    OUTTAKE_PROCESSOR(-6),
    EJECT_CORAL(-3);

    public final double setpointVolts;

    private Setpoints(double setpointVolts) {
      this.setpointVolts = setpointVolts;
    }

    public double get() {
      return this.setpointVolts;
    }
  };

  public static final double kPositionTolerance = 3;

  static {
    kClawConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kInverted);
  }
}
