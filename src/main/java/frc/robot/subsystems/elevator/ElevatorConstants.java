package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorConstants {
    //TODO: Set Constants

    public static int kMotorId = 0;
    public static boolean kInverted = false;
    public static int kCurrentLimit = 80;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static double kEncConversionFactor;
    public static int kEncCPR;

    public static double kP;
    public static double kI;
    public static double kD;
    public static double FF;
    
    public static double kMaxHeight;
    public static double kMinHeight;
}
