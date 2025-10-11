package frc.robot.subsystems.claw.claw;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.debugging.LoggedTunableNumber;

public class ClawConstants {

    public static final int kClawID = 43;
    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 60;
    public static final int kSecondaryLimit = 80;
    public static final boolean kInverted = true;
    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final LoggedTunableNumber holdCoralvoltage = new LoggedTunableNumber("Intake/HoldCoralVolt", 0.25);
    public static final double coralDetectionCutoff = 0.04;

    public static MotorConfiguration motorConfiguration = 
        new MotorConfiguration(
            kClawID, 
            kMotorType, 
            kIdleMode,
            kInverted, 
            kCurrentLimit, 
            kSecondaryLimit);

    public static ControllerConfig controllerConfig =
        new ControllerConfig(
            kP, 
            kI, 
            kD);

    public record MotorConfiguration(
        int motorID, 
        MotorType motorType, 
        IdleMode idleMode,
        boolean inverted, 
        int smartCurrentLimit,
        int secondaryCurrentLimit){}  

    public record ControllerConfig(double kP, double kI, double kD){}

    public enum RollerSpeed {
        // Coral Values
        INTAKE_CORAL(() -> 6.0),
        HOLD_CORAL(() -> 0.25),
        OUTTAKE_REEF(() -> -0.6),
        OUTTAKE_L1(() -> -3),
        EJECT_CORAL(() -> -3),

        // Algae Values
        GROUND_ALGAE(() -> 10.0),
        INTAKE_ALGAE(() -> 12.0),
        HOLD_ALGAE(() -> 7),
        EJECT_ALGAE(() -> -12),
        OUTTAKE_PROCESSOR(() -> -6),
        SCORE_BARGE(() -> -12.0);

        public final DoubleSupplier setVolts;

        private RollerSpeed(DoubleSupplier pSetVolts) {
        this.setVolts = pSetVolts;
        }

        public double get() {
        return this.setVolts.getAsDouble();
        }
    };
    
}
