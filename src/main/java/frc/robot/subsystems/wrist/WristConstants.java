package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class WristConstants {
  public static int kMotorID = 42;
  public static int kEncoderPort = 1;

  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static boolean kMotorInverted = true;

  public static int kCurrentLimit = 60; // CHANGED: 80
  public static double kP = 0.2; // 0.0015 without coral
  public static double kD = 0.005;

  public static double kMaxAcceleration = 1000;
  public static double kMaxVelocity = 800;
  public static double kTolerance = 3.0;

  public static double kForwardSoftLimit = 90;
  public static double kReverseSoftLimit = -89;
  public static double kElevatorDownLimit = -30;
  public static double throwAlgaePos = 60;

  public static double kEncoderOffsetDeg = -152.0; // 81.9;
  public static boolean kEncoderInverted = true;

  public static double kPositionConversionFactor = 360.0;
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

  public static double kS = 0.0;
  public static double kG = 0.31; // 0.29 without coral
  public static double kV = 0.0; // 0.78 without coral
  public static double kA = 0.0;

  public static final SparkMaxConfig kWristConfig = new SparkMaxConfig();

  public enum Setpoints {
    BOTTOM(0),
    INTAKE(-87),
    HPINTAKE(0),
    L1(-35),
    L2(48),
    L3(48),
    L4(65),

    SCORE(13),
    L2SCORE(11),
    BARGE(71),
    L2ALGAE(-28),
    L3ALGAE(-13),
    HOLD_ALGAE(0),
    THROW_ALGAE(90),
    CLIMB(-15),
    GROUNDALGAE(-35);

    public final double setpoint;

    private Setpoints(double setpoint) {
      this.setpoint = setpoint;
    }

    public double getPos() {
      return this.setpoint;
    }
  };

  static {
    kWristConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kMotorInverted);

    kWristConfig
        .absoluteEncoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .zeroOffset(0);
  }
}
