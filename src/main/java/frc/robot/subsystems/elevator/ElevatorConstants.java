package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConstants {
  public static int kMotorID = 41;
  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static boolean kInverted = true;
  public static int kCurrentLimit = 60; // CHANGED: 80
  public static double kP = 1.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double kMaxAcceleration = 950; // 250;
  public static double kMaxVelocity = 2400; // 500;
  public static double kTolerance = 1;

  public static double kForwardSoftLimit = 55;
  public static double kReverseSoftLimit = 0;
  public static double kReverseNoDieLimit = 21;
  public static double throwAlgaePos = 40;

  public static double kS = 0.0;
  public static double kG = 0.6;
  public static double kV = 0.0; // 4.49;
  public static double kA = 0.0; // 0.2;

  public static double kPositionConversionFactor = 1.21875; // 1.0 / kDrumCircumference
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS
  public static double kHighCutoff = 45;

  public static final SparkMaxConfig kElevatorConfig = new SparkMaxConfig();

  public enum Setpoints {
    BOTTOM(0),
    HPINTAKE(2),
    PREINTAKE(25.0),
    POSTINTAKE(18.0),
    GROUNDALGAE(8),
    L1(27),
    L2(7),
    L3(25.0),
    L4(48),
    SCORE(20),
    BARGE(55),
    L2ALGAE(38),
    L3ALGAE(53),
    HOLD_ALGAE(5),
    PreClimb(40),
    Climb(0.0);

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
