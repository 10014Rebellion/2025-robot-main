package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawConstants {

  public static class Claw {
    public static int kLeftClawID = 43;
    public static int kRightClawID = 44;
    public static int kEncoderDIOPort = 1;
    public static double kEncoderOffset = 48.5; // TODO: TUNE

    public static int kEncoderOpenPosition = 0; // TODO: Tune

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static int kCurrentLimit = 80;
    public static double kP = 0.01; // TODO: Configure me!
    public static double kD = 0.0; // TODO: Configure me!
    public static double kVelocityFF = 0.0; // TODO: Configure me!

    public static double kMaxAcceleration = 1000;
    public static double kMaxVelocity = 10000;
    public static double kTolerance = 1;

    public static double kForwardSoftLimit = 10014;
    public static double kReverseSoftLimit = 0;

    public static double kEncoderOffsetRev = 0.18528; // In revolutions

    public static double kGearRatio = 1.6 / 1;

    public static double kPositionConversionFactor =
        kGearRatio
            * 360.0; // (Drum Circumference * Final Gear Ratio) / One Encoder Revolution (if its 1:1
    // with motor shaft)  // TODO: Configure me!
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

    public static final SparkMaxConfig kClawConfig = new SparkMaxConfig();
    public static boolean hasCoral = true;
    public static boolean periodicHasCoral = false;

    public enum ClawRollerVolt {
      INTAKE_CORAL(1),
      INTAKE_ALGAE(3),
      HOLD_ALGAE(0.5),
      OUTTAKE_REEF(-0.5),
      OUTTAKE_BARGE(-8),
      EJECT_CORAL(-3);

      public final double voltage;

      private ClawRollerVolt(double voltage) {
        this.voltage = voltage;
      }

      public double get() {
        return this.voltage;
      }
    };

    public static final double positionTolerance = 3;
    // Max 123
    public enum ClawOpenPositions {
      NO_CORAL(0),
      OPEN(26), // Maximum the claw goes when the coral is going in, when its tangent to the front 2
      // wheels
      HAS_CORAL(23),
      MAX(33);

      public final double position;

      private ClawOpenPositions(double position) {
        this.position = position;
      }

      public double get() {
        return this.position;
      }
    };

    static {
      kClawConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);

      // kWristConfig
      //     .softLimit
      //     .forwardSoftLimit(kForwardSoftLimit)
      //     .reverseSoftLimit(kReverseSoftLimit);

      kClawConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .zeroOffset(kEncoderOffsetRev);

      // kWristConfig
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     .pidf(kP, 0.0, kD, kVelocityFF)
      //     .outputRange(-1, 1);

      // kWristConfig
      //     .closedLoop
      //     .maxMotion
      //     .maxVelocity(kMaxVelocity)
      //     .maxAcceleration(kMaxAcceleration)
      //     .allowedClosedLoopError(kTolerance);
    }
  }

  public static class Wrist {
    public static int kMotorID = 42;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static int kCurrentLimit = 80;
    public static double kP = 0.12; // TODO: Configure me!
    public static double kD = 0.0; // TODO: Configure me!
    public static double kVelocityFF = 0.0; // TODO: Configure me!

    public static double kMaxAcceleration = 100;
    public static double kMaxVelocity = 100;
    public static double kTolerance = 5;

    public static double kForwardSoftLimit = 70;
    public static double kReverseSoftLimit = -62;
    public static double kGearRatio = 50.0 / 84.0;

    public static double kEncoderOffsetRev = 0.0623246; // In revolutions

    public static double kPositionConversionFactor = kGearRatio * 360.0;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

    public static double kS = 0.0;
    public static double kG = 0.21;
    public static double kV = 2.15;
    public static double kA = 0.0;

    public static final SparkMaxConfig kWristConfig = new SparkMaxConfig();

    public enum Positions {
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
      CLIMB(57); // TO DO: redo these values

      public final double position;

      private Positions(double position) {
        this.position = position;
      }

      public double getPos() {
        return this.position;
      }
    };

    static {
      kWristConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);

      // kWristConfig
      //     .softLimit
      //     .forwardSoftLimit(kForwardSoftLimit)
      //     .reverseSoftLimit(kReverseSoftLimit);

      kWristConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .zeroOffset(kEncoderOffsetRev);

      // kWristConfig
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     .pidf(kP, 0.0, kD, kVelocityFF)
      //     .outputRange(-1, 1);

      // kWristConfig
      //     .closedLoop
      //     .maxMotion
      //     .maxVelocity(kMaxVelocity)
      //     .maxAcceleration(kMaxAcceleration)
      //     .allowedClosedLoopError(kTolerance);
    }
  }
}
