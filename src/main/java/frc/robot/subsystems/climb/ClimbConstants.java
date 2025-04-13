package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbConstants {
  public class Grabber {
    public static int kMotorID = 62;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kMotorInverted = false;

    public static int kCurrentLimit = 60;

    public static final SparkMaxConfig kClimbConfig = new SparkMaxConfig();

    public enum VoltageSetpoints {
      PULL_IN(8),
      SPIN_FREE(2);

      public final double setpoint;

      private VoltageSetpoints(double setpoint) {
        this.setpoint = setpoint;
      }

      public double getVolts() {
        return this.setpoint;
      }
    };

    static {
      kClimbConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kMotorInverted);
    }
  }

  public class Pulley {
    public static int kMotorID = 61;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kMotorInverted = true;

    public static boolean encoderInverted = false;
    public static double encoderZeroOffset = 0.3297160;
    public static double encoderPositionFactor = 360.0;
    public static double encoderVelocityFactor = 360.0;

    public static int kCurrentLimit = 60;

    public static final SparkMaxConfig kClimbConfig = new SparkMaxConfig();

    public enum VoltageSetpoints {
      ASCEND(12),
      DESCEND(-12),
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
      EXTENDED(0.0),
      CLIMBED(95.0),
      STOWED(150.0);
  
      public final double setpoint;
  
      private Setpoints(double setpoint) {
        this.setpoint = setpoint;
      }
  
      public double getPos() {
        return this.setpoint;
      }
    };
    static {
      kClimbConfig
        .idleMode(kIdleMode)
        .smartCurrentLimit(kCurrentLimit)
        .inverted(kMotorInverted);
      kClimbConfig.absoluteEncoder
        .inverted(encoderInverted)
        .zeroOffset(encoderZeroOffset)
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);
    }
  }
}
