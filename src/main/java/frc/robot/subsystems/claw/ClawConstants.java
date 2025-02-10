package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawConstants {

  public static class Claw {
    public static int kLeftClawID = 43;
    public static int kRightClawID = 44;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static int kCurrentLimit = 80;
    public static double kP = 0.01; // TODO: Configure me!
    public static double kD = 0.0; // TODO: Configure me!
    public static double kVelocityFF = 0.0; // TODO: Configure me!

    public static double kMaxAcceleration = 100;
    public static double kMaxVelocity = 100;
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
    public static double kP = 0.01; // TODO: Configure me!
    public static double kD = 0.0; // TODO: Configure me!
    public static double kVelocityFF = 0.0; // TODO: Configure me!

    public static double kMaxAcceleration = 100;
    public static double kMaxVelocity = 100;
    public static double kTolerance = 1;

    public static double kForwardSoftLimit = 60;
    public static double kReverseSoftLimit = -45;
    public static double kGearRatio = 50.0 / 84.0;

    public static double kEncoderOffsetRev = 0.0623246; // In revolutions

    public static double kPositionConversionFactor = kGearRatio * 360.0;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

    public static double kS = 0.0;
    public static double kG = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static final SparkMaxConfig kWristConfig = new SparkMaxConfig();

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
