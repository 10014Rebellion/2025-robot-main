package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int kMotorID = 41;
  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static int kCurrentLimit = 50;
  public static double kP = 0.0; // TODO: Configure me!
  public static double kD = 0.0; // TODO: Configure me!
  public static double kPositionFF = 0.0; // TODO: Configure me!
  public static double kVelocityFF = 0.0; // TODO: Configure me!


  public static double kDrumDiameterM = Units.inchesToMeters(1.5); // Sprocket diameter, should be around 1.5in  // TODO: Configure me! 
  public static double kDrumCircumference = kDrumDiameterM * Math.PI;

  public static double kGearRatio = 1; // TODO: Configure me!
  
  public static double kPositionConversionFactor = ((kDrumDiameterM * Math.PI) * kGearRatio) / 1.0; // (Drum Circumference * Final Gear Ratio) / One Encoder Revolution (if its 1:1 with motor shaft)  // TODO: Configure me! 
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

  public final static SparkMaxConfig kElevatorConfig = new SparkMaxConfig();
  
  static {
    kElevatorConfig
      .idleMode(kIdleMode)
      .smartCurrentLimit(kCurrentLimit);
    
    kElevatorConfig.encoder
      .positionConversionFactor(kPositionConversionFactor)
      .velocityConversionFactor(kVelocityConversionFactor);
      
    kElevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kP, 0.0, kD, kVelocityFF)
      .outputRange(-1, 1);
  }
}
