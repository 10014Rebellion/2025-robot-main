package frc.robot.subsystems.elevatorPivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorPivotConstants {
  public static int kMotorID = 45;
  public static int kCurrentLimit = 20;
  public static MotorType kMotorType = MotorType.kBrushed;
  public static IdleMode kIdleMode = IdleMode.kBrake;

  public static final SparkMaxConfig kPivotConfig = new SparkMaxConfig();
  public static final double kPositionConversionFactor = 360;

  enum Positions {
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
