package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawConstants {

  public static final int kClawID = 43;
  public static final int kBeamBreakDIOPort = 2;
  // public static final double kEncoderOffset = 48.5; // TODO: TUNE

  public static final MotorType kMotorType = MotorType.kBrushless;
  public static final IdleMode kIdleMode = IdleMode.kBrake;
  public static final int kCurrentLimit = 80;
  public static final boolean kInverted = true;
  public static final double kP = 0.01; // TODO: Configure me!
  public static final double kD = 0.0; // TODO: Configure me!

  public static final SparkMaxConfig kClawConfig = new SparkMaxConfig();

  public enum RollerSpeed {
    INTAKE_CORAL(6.0),
    HOLD_CORAL(0.5),
    INTAKE_ALGAE(6.0),
    HOLD_ALGAE(5),
    OUTTAKE_REEF(-0.6),
    REVERSE_REEF(-0.65),
    OUTTAKE_BARGE(-8),
    OUTTAKE_PROCESSOR(-6),
    OUTTAKE_L1(-3),
    EJECT_CORAL(-3);

    public final double setVolts;

    private RollerSpeed(double pSetVolts) {
      this.setVolts = pSetVolts;
    }

    public double get() {
      return this.setVolts;
    }
  };

  public static final double kPositionTolerance = 3;

  static {
    kClawConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kInverted);
  }
}
