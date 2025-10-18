package frc.robot.subsystems.intake.IntakeRoller;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeRollerConstants {
    public static int kMotorID = 54;

    public static NeutralModeValue kIdleMode = NeutralModeValue.Brake;
    public static boolean kInverted = true;
    public static double kIntakeSpeed = 12;

    public static int kSmartCurrentLimit = 60;
    public static int kSecondaryCurrentLimit = 75;

    public static IntakeRollerHardware intakeRollersHardware = new IntakeRollerHardware(kMotorID, "drivetrain");

    public static IntakeRollerConfiguration motorConfiguration = new IntakeRollerConfiguration(
        kSmartCurrentLimit, 
        kSecondaryCurrentLimit, 
        kIdleMode, 
        kInverted);

    public record IntakeRollerHardware(
        int kMotorPort,
        String kCanBus
    ){}

    public record IntakeRollerConfiguration(
        int kSmartLimit,
        int kSecondaryLimit,
        NeutralModeValue kIdleMode,
        boolean kInverted
    ){}
}
