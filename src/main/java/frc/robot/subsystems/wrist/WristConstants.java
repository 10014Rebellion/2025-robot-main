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

  public static int kCurrentLimit = 60;

  public static double k0P = 0.22;
  public static double k0I = 0.0;
  public static double k0D = 0.01;

  public static double k0MaxAcceleration = 800; // was 500. k0inda slow
  public static double k0MaxVelocity = 800;

  public static double k0S = 0.02;
  public static double k0G = 0.4;
  public static double k0V = 0.003;
  public static double k0A = 0.0;

  // SLOT 1: BARGE
  public static double k1P = 0.22;
  public static double k1I = 0.0;
  public static double k1D = 0.01;

  public static double k1MaxAcceleration = 1100; // was 500. k1inda slow
  public static double k1MaxVelocity = 800;

  public static double k1S = 0.02;
  public static double k1G = 0.4;
  public static double k1V = 0.003;
  public static double k1A = 0.0;

  public static double k2P = 0.22;
  public static double k2I = 0.0;
  public static double k2D = 0.01;

  public static double k2MaxAcceleration = 650; // was 500. k2inda slow
  public static double k2MaxVelocity = 400;

  public static double k2S = 0.02;
  public static double k2G = 0.4;
  public static double k2V = 0.003;
  public static double k2A = 0.0;

  public static double kTolerance = 1.5;

  public static double kForwardSoftLimit = 90;
  public static double kReverseSoftLimit = -89;
  public static double kElevatorDownLimit = -30;
  public static double throwAlgaePos = 50;

  public static double kEncoderOffsetDeg = -162.95;
  public static boolean kEncoderInverted = true;

  public static double kPositionConversionFactor = 360.0;
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

  public static final SparkMaxConfig kWristConfig = new SparkMaxConfig();

  public enum Setpoints {
    BOTTOM(0),
    INTAKE(-88),
    HPINTAKE(0),
    L1(-30),
    L2(46 + 6),
    L3(46 + 6),
    L4(65),

    SCORE(11),
    L2SCORE(9),
    BARGE(71),
    L2ALGAE(-28),
    L3ALGAE(-16),
    HOLD_ALGAE(0),
    THROW_ALGAE(90),
    CLIMB(-15),
    GROUNDALGAE(-31),
    STOWED(90);

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

    kWristConfig.absoluteEncoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .zeroOffset(0);
  }
}
