package frc.robot.subsystems.intake.Indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerConstants {

    public static int kMotorID = 55;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static int kSmartCurrentLimit = 60;
    public static int kSecondaryCurrentLimit = 85;
    public static boolean kInverted = true;

    public static double kIntakeVolts = 9.5;
    public static double kIntakeVoltsSlow = 2.5;
    public static double kIntakeVoltsHold = 1.0;
    public static double kOuttakeVolts = -6;

    public static double kIntakeStuckTime = 0.5;
    public static double kIntakeReverseTime = 0.25;

    public static final SparkMaxConfig kIndexerConfig = new SparkMaxConfig();

    public static IndexerHardware indexerHardware = new IndexerHardware(kMotorID, kMotorType);

    public static IndexerConfiguration motorConfiguration = new IndexerConfiguration(
        kSmartCurrentLimit, 
        kSecondaryCurrentLimit, 
        kIdleMode, 
        kInverted);

    public record IndexerHardware(
        int kMotorPort,
        MotorType kMotorType
    ){}

    public record IndexerConfiguration(
        int kSmartLimit,
        int kSecondaryLimit,
        IdleMode kIdleMode,
        boolean kInverted
    ){}
    
}
