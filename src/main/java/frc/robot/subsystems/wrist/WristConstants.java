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
  public static double kP = 0.13;
  public static double kD = 0.0;

  public static double kMaxAcceleration = 900; // 1000 too fast
  public static double kMaxVelocity = 1000;
  public static double kTolerance = 5;

  public static double kForwardSoftLimit = 145;
  public static double kReverseSoftLimit = -89;

  public static double kEncoderOffsetDeg = -149.0; // 81.9;
  public static boolean kEncoderInverted = true;

  public static double kPositionConversionFactor = 360.0;
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

  public static double kS = 0.0;
  public static double kG = 0.3;
  public static double kV = 2.02;
  public static double kA = 0.02;

  public static final SparkMaxConfig kWristConfig = new SparkMaxConfig();

  public enum Setpoints {
    BOTTOM(0),
    INTAKE(-87),
    HPINTAKE(0),
    L1(-27),
    L2(56),
    L3(56),
    L4(65),
    REVERSEL4(135),
    NORM_LOLI_L4(65),

    SCORE(10),
    BARGE(71),
    L2ALGAE(-24),
    L3ALGAE(-12),
    HOLD_ALGAE(0),
    CLIMB(45),
    GROUNDINTAKE(-20),
    GROUNDPOSTINTAKE(-10),
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
