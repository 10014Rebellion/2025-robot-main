package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawConstants {

  public static final int kClawID = 43;
  public static final int kBeamBreakDIOPort = 2;
  public static final MotorType kMotorType = MotorType.kBrushless;
  public static final IdleMode kIdleMode = IdleMode.kBrake;
  public static final int kCurrentLimit = 60;
  public static final boolean kInverted = true;
  public static final double kP = 0.01;
  public static final double kD = 0.0;

  public static final SparkMaxConfig kClawConfig = new SparkMaxConfig();

  public enum RollerSpeed {
    // Coral Values
    INTAKE_CORAL(6.0),
    HOLD_CORAL(0.35),
    OUTTAKE_REEF(-0.6),
    OUTTAKE_L1(-3),
    EJECT_CORAL(-3),

    // Algae Values
    INTAKE_ALGAE(12.0),
    HOLD_ALGAE(7),
    EJECT_ALGAE(-12),
    OUTTAKE_PROCESSOR(-6),
    SCORE_BARGE(-12.0);

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
