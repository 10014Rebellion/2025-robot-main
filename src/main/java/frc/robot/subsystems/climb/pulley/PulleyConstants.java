package frc.robot.subsystems.climb.pulley;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class PulleyConstants {
    public class Pulley {
        public static int kMotorID = 61;

        public static MotorType kMotorType = MotorType.kBrushless;
        public static IdleMode kIdleMode = IdleMode.kBrake;
        public static boolean kMotorInverted = true;


        public static boolean encoderInverted = false;
        public static double encoderZeroOffset = 0.3297160;
        public static double encoderPositionFactor = 360.0;
        public static double encoderVelocityFactor = 360.0;
        public static double kTolerance = 3.0;

        public static int kSmartLimit = 60;
        public static int kSecondaryLimit = 80;

        public static final SparkMaxConfig kClimbConfig = new SparkMaxConfig();

        public enum VoltageSetpoints {
            // ASCEND(0.0),
            GO(7),
            GO_FRICKEN_FAAAST(7.5), //voltage coming back in! GOING MAX THROTTLEEEE
            GO_SLOW(2.8),
            STOP(0.0);

            public final double setpoint;

            private VoltageSetpoints(double setpoint) {
                this.setpoint = setpoint;
            }

            public double getVolts() {
                return this.setpoint;
            }
        };

        public enum Setpoints {
            EXTENDED(5.0),
            STARTROLLING(28.0), // angle of the climb where it start intaking the cage
            SLOW_DOWN(70.0), 
            CLIMBED(95.0),
            STOWED(149.0);

            public final double setpoint;

            private Setpoints(double setpoint) {
                this.setpoint = setpoint;
            }

            public double getPos() {
                return this.setpoint;
            }
        };

        public record PulleyConfiguration(
            int kSmartLimit,
            int kSecondaryLimit,
            IdleMode kIdleMode,
            boolean kInverted){}

        public record EncoderConfiguration(
            boolean kInverted,
            Rotation2d offset,
            double kPosistionFactor,
            double kVelocityFactor,
            double kTolerance
        ){}

        public record PulleyHardware(
            int kMotorID,
            MotorType kMotorType){}
      
        public static PulleyConfiguration motorConfiguration = new PulleyConfiguration(kSmartLimit, kSecondaryLimit, kIdleMode, kMotorInverted);

        public static EncoderConfiguration encoderConfiguration = new EncoderConfiguration(encoderInverted, Rotation2d.fromRotations(encoderZeroOffset), encoderPositionFactor, encoderVelocityFactor, kTolerance);
      
        public static PulleyHardware pulleyHardware = new PulleyHardware(kMotorID, kMotorType);

        // static {
        // kClimbConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kMotorInverted);
        // kClimbConfig
        //     .absoluteEncoder
        //     .inverted(encoderInverted)
        //     .zeroOffset(encoderZeroOffset)
        //     .positionConversionFactor(encoderPositionFactor)
        //     .velocityConversionFactor(encoderVelocityFactor);
        // }

    }
}