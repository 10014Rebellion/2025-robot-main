package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double mPosition;
    public double mVelocity;
    public double mAppliedVolts;
    public double mAppliedCurrent;
  }

  public default void updateInputs(ElevatorIOInputs pInputs) {}

  public default void setVoltage(double pVoltage) {}

  public default void setPercentOutput(double pOutput) {}

  public default void disable() {}

  // TODO: Fix this
  public default double getExtension() {
    return 0;
  }
}
