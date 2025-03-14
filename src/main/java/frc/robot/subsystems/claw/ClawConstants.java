package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawConstants {

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

    public enum Setpoints {
      INTAKE_CORAL(1),
      INTAKE_ALGAE(3),
      HOLD_ALGAE(0.5),
      OUTTAKE_REEF(-0.5),
      OUTTAKE_BARGE(-8),
      OUTTAKE_PROCESSOR(-6),
      EJECT_CORAL(-3);

      public final double setpointVolts;

      private Setpoints(double setpointVolts) {
        this.setpointVolts = setpointVolts;
      }

      public double get() {
        return this.setpointVolts;
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

      kClawConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .zeroOffset(kEncoderOffsetRev);

    }
  
}
