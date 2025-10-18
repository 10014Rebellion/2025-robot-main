package frc.robot.subsystems.intake.IntakePivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakePivotConstants {
    public static int kMotorID = 53;

    public static int kIntakeSpeed = 12;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kMotorInverted = false;
    public static boolean kEncoderInverted = true;

    public static int kSmartCurrentLimit = 60;
    public static int kSecondaryCurrentLimit = 75;

    public static double kPositionConversionFactor = 360.0;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static double kEncoderOffsetRev = 0.3345553;

    public static final SparkMaxConfig kPivotConfig = new SparkMaxConfig();

    public static double kTolerance = 3;
    public static double kP = 0.2;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kMaxVelocity = 300; // Theoretical max: 1555
    public static double kMaxAcceleration = 500; // Theoretical max: 16179 deg/s^2

    public static double kForwardSoftLimit = 90.0; // TO DO: CONFIGURE ME!
    public static double kReverseSoftLimit = 2.0; // TO DO: CONFIGURE ME!

    public static double kS = 0.0; // 0.34
    public static double kG = 0.19; // 0.3
    public static double kV = 2.4; // 0.43
    public static double kA = 0.08; // 0.06

    public enum Setpoints {
      STOWED(90.0),
      ALGAEINTAKE(70),
      INTAKING(3.0);
      public final double position;

      private Setpoints(double position) {
        this.position = position;
      }

      public double getPos() {
        return this.position;
      }
    };


    public static IntakePivotHardware intakePivotHardware = new IntakePivotHardware(kMotorID, kMotorType);

    public static IntakePivotEncoderHardware encoderHardware = new IntakePivotEncoderHardware(kEncoderOffsetRev, kEncoderInverted);

    public static IntakePivotConfiguration motorConfiguration = new IntakePivotConfiguration(
        kSmartCurrentLimit, 
        kSecondaryCurrentLimit, 
        kIdleMode, 
        kMotorInverted, 
        kForwardSoftLimit, 
        kReverseSoftLimit, 
        kPositionConversionFactor, 
        kVelocityConversionFactor);

    ControllerConfiguration controller0 = new ControllerConfiguration(
        kP, 
        kI, 
        kD, 
        kS, 
        kV, 
        kA, 
        kG, 
        kMaxVelocity, 
        kMaxAcceleration, 
        kTolerance);


    public record IntakePivotHardware(
        int kMotorPort,
        MotorType kMotorType
    ){}

    public record IntakePivotEncoderHardware(
        double kEncoderOffset,
        boolean kEncoderInverted
    ){}

    public record IntakePivotConfiguration(
        int kSmartLimit,
        int kSecondaryLimit,
        IdleMode kIdleMode,
        boolean kInverted,
        double kForwardSoftLimit,
        double kReverseSoftLimit,
        double kPositionConversionFactor,
        double kVelocityConversionFactor
    ){}

    public record ControllerConfiguration(
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kG,
        double kA,
        double kMaxVelo,
        double kMaxAccel,
        double kTolerance
    ){}
}
