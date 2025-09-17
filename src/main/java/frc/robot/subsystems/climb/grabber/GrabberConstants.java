package frc.robot.subsystems.climb.grabber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class GrabberConstants {

  public class Grabber {
    public static int kMotorID = 62;
    public static int kClimbBeamBreakID = 6;
    public static MotorType kMotorType = MotorType.kBrushless;

    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kMotorInverted = true;
    public static int kSmartCurrentLimit = 60;
    public static int kSecondaryCurrentLimit = 80;


    public enum VoltageSetpoints {
      PULL_IN(4.0);
      // SPIN_FREE(2);

      public final double setpoint;

      private VoltageSetpoints(double setpoint) {
        this.setpoint = setpoint;
      }

      public double getVolts() {
        return this.setpoint;
      }
    };

    public record GrabberConfiguration(
        int kSmartLimit,
        int kSecondaryLimit,
        IdleMode kIdleMode,
        boolean inverted){}
  
    public static GrabberConfiguration motorConfiguration = new GrabberConfiguration(kSmartCurrentLimit, kSecondaryCurrentLimit, kIdleMode, kMotorInverted);

    public record GrabberHardware(
        int kMotorID,
        int kBeamBreakID,
        MotorType kMotorType){}
  
    public static GrabberHardware grabberHardware = new GrabberHardware(kMotorID, kClimbBeamBreakID, kMotorType);

}
}