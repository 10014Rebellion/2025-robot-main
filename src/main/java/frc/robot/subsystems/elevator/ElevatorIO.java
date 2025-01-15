package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double position;
    public double velocity;
    public double appliedVolts;
    public double appliedCurrent;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setPercentOutput(double output) {}
}
