package frc.robot.subsystems.wrist;

import com.revrobotics.spark.ClosedLoopSlot;
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

  // SLOT 0: Coral/Default
  public static double k0P = 0.22;
  public static double k0I = 0.0;
  public static double k0D = 0.01;

  public static double k0MaxAcceleration = 800;
  public static double k0MaxVelocity = 800;

  public static double k0S = 0.02;
  public static double k0G = 0.4;
  public static double k0V = 0.003;
  public static double k0A = 0.0;

  // SLOT 1: BARGE
  public static double k1P = 0.22;
  public static double k1I = 0.0;
  public static double k1D = 0.01;

  public static double k1MaxAcceleration = 1100; 
  public static double k1MaxVelocity = 800;

  public static double k1S = 0.02;
  public static double k1G = 0.4;
  public static double k1V = 0.003;
  public static double k1A = 0.0;

  // SLOT 2: TUNING
  public static double k2P = k0P;
  public static double k2I = k0I;
  public static double k2D = k0D;

  public static double k2MaxAcceleration = k0MaxAcceleration;
  public static double k2MaxVelocity = k0MaxVelocity;

  public static double k2S = k0S;
  public static double k2G = k0G;
  public static double k2V = k0V;
  public static double k2A = k0A;

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
    L3ALGAE(-19),
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

    kWristConfig.closedLoop
      .pid(k0P, k0I, k0D, ClosedLoopSlot.kSlot0)
      .pid(k1P, k1I, k1D, ClosedLoopSlot.kSlot1)
      .pid(k2P, k2I, k2D, ClosedLoopSlot.kSlot2)
        .feedForward
          // Coral/Default
          .kA(k0A, ClosedLoopSlot.kSlot0)
          .kS(k0S, ClosedLoopSlot.kSlot0)
          .kV(k0V, ClosedLoopSlot.kSlot0)
          .kG(k0G, ClosedLoopSlot.kSlot0)

          // Barge Toss
          .kA(k1A, ClosedLoopSlot.kSlot1)
          .kS(k1S, ClosedLoopSlot.kSlot1)
          .kV(k1V, ClosedLoopSlot.kSlot1)
          .kG(k1G, ClosedLoopSlot.kSlot1)

          // Tuning Config
          .kA(k2A, ClosedLoopSlot.kSlot2)
          .kS(k2S, ClosedLoopSlot.kSlot2)
          .kV(k2V, ClosedLoopSlot.kSlot2)
          .kG(k2G, ClosedLoopSlot.kSlot2);

    kWristConfig.closedLoop
      .maxMotion
        .cruiseVelocity(k0MaxVelocity, ClosedLoopSlot.kSlot0)
        .maxAcceleration(k0MaxAcceleration, ClosedLoopSlot.kSlot0)
        .allowedProfileError(kTolerance, ClosedLoopSlot.kSlot0)

        .cruiseVelocity(k1MaxVelocity, ClosedLoopSlot.kSlot1)
        .maxAcceleration(k1MaxAcceleration, ClosedLoopSlot.kSlot1)
        .allowedProfileError(kTolerance, ClosedLoopSlot.kSlot1)

        .cruiseVelocity(k2MaxVelocity, ClosedLoopSlot.kSlot2)
        .maxAcceleration(k2MaxAcceleration, ClosedLoopSlot.kSlot2)
        .allowedProfileError(kTolerance, ClosedLoopSlot.kSlot2);

    
  }
}
