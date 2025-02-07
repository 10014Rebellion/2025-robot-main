package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int kMotorID = 41;
  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static int kCurrentLimit = 80;
  public static double kP = 0.01; // TODO: Configure me!
  public static double kD = 0.0; // TODO: Configure me!
  public static double kVelocityFF = 0.0; // TODO: Configure me!

  public static double kMaxAcceleration = 500;
  public static double kMaxVelocity = 6000;
  public static double kTolerance = 1;

  public static double kForwardSoftLimit = 70;
  public static double kReverseSoftLimit = 0;

  public static double kDrumDiameterM = Units.inchesToMeters(2.635); // Sprocket diameter
  public static double kDrumCircumference = kDrumDiameterM * Math.PI;

  // public static double kGearRatio = 25 / 1;

  public static double kS = 0.0;
  public static double kG = 0.0;
  public static double kV = 0.0;
  public static double kA = 0.0;

  public static double kPositionConversionFactor = 1.21875; // 1.0 / kDrumCircumference
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

  public static final SparkMaxConfig kElevatorConfig = new SparkMaxConfig();

  static {
    kElevatorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);

    // kElevatorConfig
    //     .softLimit
    //     .forwardSoftLimit(kForwardSoftLimit)
    //     .reverseSoftLimit(kReverseSoftLimit);

    kElevatorConfig
        .encoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor);

    // kElevatorConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .pidf(kP, 0.0, kD, kVelocityFF)
    //     .outputRange(-1, 1);

    // kElevatorConfig
    //     .closedLoop
    //     .maxMotion
    //     .maxVelocity(kMaxVelocity)
    //     .maxAcceleration(kMaxAcceleration)
    //     .allowedClosedLoopError(kTolerance);
  }
}
