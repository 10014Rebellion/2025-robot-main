package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PivotConstants {
  public static int kMotorID = 45;
  public static int kCurrentLimit = 20;
  public static MotorType kMotorType = MotorType.kBrushed;
  public static IdleMode kIdleMode = IdleMode.kBrake;

  public static final SparkMaxConfig kPivotConfig = new SparkMaxConfig();
  public static final double kPositionConversionFactor = 360;
  public static final double kForwardSoftLimit = 50;
  public static final double kReverseSoftLimit = 0;

  public static final double kP = 1;
  public static final double kD = 0;
  public static final double kTolerance = 0.5;

  public enum Positions {
    NORMAL(14),
    CLIMB(40);
    public final double position;

    private Positions(double position) {
      this.position = position;
    }

    public double getPos() {
      return this.position;
    }
  };

  static {
    kPivotConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);
  }
}
