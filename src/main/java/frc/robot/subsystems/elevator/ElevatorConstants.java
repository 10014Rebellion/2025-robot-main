package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int kMotorID = 41;
  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static int kCurrentLimit = 80;
  public static double kP = 1.5; // TODO: Configure me!
  public static double kI = 0.0;
  public static double kD = 0.08; // TODO: Configure me!
  public static double kVelocityFF = 0.0; // TODO: Configure me!

  public static double kMaxAcceleration = 250;
  public static double kMaxVelocity = 500;
  public static double kTolerance = 5;

  public static double kForwardSoftLimit = 80;
  public static double kReverseSoftLimit = 0;

  // public static double kDrumDiameterM = Units.inchesToMeters(2.635); // Sprocket diameter
  // public static double kDrumCircumference = kDrumDiameterM * Math.PI;

  // public static double kGearRatio = 25 / 1;

  public static double kS = 0.0;
  public static double kG = 0.9;
  public static double kV = 0.0;
  public static double kA = 0.0;

  public static double kPositionConversionFactor = 1.21875; // 1.0 / kDrumCircumference
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

  public static final SparkMaxConfig kElevatorConfig = new SparkMaxConfig();

  public enum Positions {
    BOTTOM(0),
    PREINTAKE(25),
    POSTINTAKE(15),
    L1(0),
    L2(25),
    L3(50),
    L4(75),
    SCORE(20),
    BARGE(0);

    public final double position;

    private Positions(double position) {
      this.position = position;
    }

    public double getPos() {
      return this.position;
    }
  };

  static {
    kElevatorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);

    kElevatorConfig
        .encoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor);

    kElevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, 0.0, kD, kVelocityFF)
        .outputRange(-1, 1);

    kElevatorConfig
        .closedLoop
        .maxMotion
        .maxVelocity(kMaxVelocity)
        .maxAcceleration(kMaxAcceleration)
        .allowedClosedLoopError(kTolerance);
  }
}
