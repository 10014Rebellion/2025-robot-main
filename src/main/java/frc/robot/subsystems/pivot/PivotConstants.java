package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PivotConstants {
  public static int kMotorID = 45;
  public static int kCurrentLimit = 20;
  public static MotorType kMotorType = MotorType.kBrushed;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static boolean kInverted = false;

  public static final SparkMaxConfig kPivotConfig = new SparkMaxConfig();

  public static final double kForwardSoftLimit = 50;
  public static final double kReverseSoftLimit = 0;

  public static final double kPositionConversionFactor = 360.0;
  public static final double kVelocityConversionFactor = 1.0 / 60.0;

  public static final double kP = 0.01; // TODO: TUNE ME
  public static final double kD = 0;
  public static final double kMaxVelocity = 0;
  public static final double kMaxAcceleration = 0;

  public static final double kTolerance = 0.5;

  public static final double kS = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public enum Setpoints {
    NORMAL(14),
    CLIMB(40);
    public final double setpoint;

    private Setpoints(double setpoint) {
      this.setpoint = setpoint;
    }

    public double getPos() {
      return this.setpoint;
    }
  };

  static {
    kPivotConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kInverted);

    kPivotConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
  }
}
