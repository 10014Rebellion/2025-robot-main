package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorConstants {
  public static int kMotorId = 0; // TODO: Configure me!
  public static boolean kInverted = false; // TODO: Configure me!
  public static int kCurrentLimit = 80;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static double kEncConversionFactor = 0.0; // TODO: Configure me!
  public static int kEncCPR = 0; // TODO: Configure me!

  public static double kP = 0.0; // TODO: Configure me!
  public static double kI = 0.0;
  public static double kD = 0.0; // TODO: Configure me!
  public static double FF = 0.0;
  public static double kControllerTolerance = 1.0; // TODO: Configure me!

  public static double kMaxHeight = 0.0;
  public static double kMinHeight = 0.0;
  public static double kL1Height = 0.0;
  public static double kL2Height = 0.0;
  public static double kL3Height = 0.0;
  public static double kL4Height = 0.0;
  public static double kProcessorHeight = 0.0;
  public static double kNetHeight = 0.0;
}
