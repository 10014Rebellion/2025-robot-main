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

  public static int kCurrentLimit = 80;
  public static double kP = 0.12;
  public static double kD = 0.0;

  public static double kMaxAcceleration = 100;
  public static double kMaxVelocity = 100;
  public static double kTolerance = 5;

  public static double kForwardSoftLimit = 90;
  public static double kReverseSoftLimit = -85;

  public static double kEncoderOffsetDeg = 145;
  public static boolean kEncoderInverted = true;

  public static double kPositionConversionFactor = 360.0;
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

  public static double kS = 0.0;
  public static double kG = 0.21;
  public static double kV = 2.15;
  public static double kA = 0.0;

  public static final SparkMaxConfig kWristConfig = new SparkMaxConfig();

  public enum Setpoints {
    BOTTOM(0),
    INTAKE(-65),
    L1(20),
    L2(50),
    L3(70),
    L4(50),
    SCORE(-50),
    BARGE(70),
    L2ALGAE(-12),
    L3ALGAE(-12),
    HOLD_ALGAE(14),
    CLIMB(57);

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
