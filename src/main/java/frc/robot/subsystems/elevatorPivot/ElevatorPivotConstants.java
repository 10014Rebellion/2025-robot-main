package frc.robot.subsystems.elevatorPivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorPivotConstants {
    public static int kMotorID = 0;
    public static int kCurrentLimit = 5;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;

    public static final SparkMaxConfig kPivotConfig = new SparkMaxConfig();

    static {
        kPivotConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);
    }
}
