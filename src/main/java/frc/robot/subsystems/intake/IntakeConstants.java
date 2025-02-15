package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConstants {
  public static class Funnel {
    public static int kMotorID = 55;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static int kCurrentLimit = 40;

    public static final SparkMaxConfig kFunnelConfig = new SparkMaxConfig();

    static {
      kFunnelConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);
    }
  }
}
