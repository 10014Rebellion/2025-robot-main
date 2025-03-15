package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConstants {
  public static int kMotorID = 41;
  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static boolean kInverted = true;
  public static int kCurrentLimit = 80;
  public static double kP = 1.5;
  public static double kI = 0.0;
  public static double kD = 0.08;

  public static double kMaxAcceleration = 250;
  public static double kMaxVelocity = 500;
  public static double kTolerance = 1;

  public static double kForwardSoftLimit = 85;
  public static double kReverseSoftLimit = 0;

  public static double kS = 0.0;
  public static double kG = 0.9;
  public static double kV = 0.0;
  public static double kA = 0.0;

  public static double kPositionConversionFactor = 1.21875; // 1.0 / kDrumCircumference
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

  public static final SparkMaxConfig kElevatorConfig = new SparkMaxConfig();

  public enum Setpoints {
    BOTTOM(0),
    PREINTAKE(35),
    POSTINTAKE(25),
    GROUNDINTAKE(0.5),
    L1(45),
    L2(10),
    L3(37.5),
    L4(83),
    SCORE(20),
    BARGE(80),
    L2ALGAE(44.5),
    L3ALGAE(67.5),
    HOLD_ALGAE(7);

    public final double setpoint;

    private Setpoints(double setpoint) {
      this.setpoint = setpoint;
    }

    public double getPos() {
      return this.setpoint;
    }
  };

  static {
    kElevatorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kInverted);

    kElevatorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
  }
}
