package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbConstants {
  public class Grabber {
    public static int kMotorID = 0; // TODO: Tune this

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kMotorInverted = false;

    public static int kCurrentLimit = 80;

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
    public static int kMotorID = 0; // TODO: Tune this

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kMotorInverted = false;

    public static int kCurrentLimit = 80;

    public static final SparkMaxConfig kClimbConfig = new SparkMaxConfig();

    public enum VoltageSetpoints {
      ASCEND(5),
      DESCEND(-5);

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
}
