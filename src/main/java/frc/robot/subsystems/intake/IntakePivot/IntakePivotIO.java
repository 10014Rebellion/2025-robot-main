
package frc.robot.subsystems.intake.IntakePivot;

import org.littletonrobotics.junction.AutoLog;

/** The intake subsystem's hardware interface. */
public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public boolean isMotorConnected = false;

    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double motorOutput = 0.0;
    public double position = 0.0;
  }

  /**
   * Write data from the hardware to the inputs object
   * 
   * @param inputs The inputs object
   */
  public default void updateInputs(IntakePivotIOInputs inputs) {};

  public default void stopIfLimitIntakePivot() {};

  /**
   * @param volts The voltage that should be applied to the motor from -12 to 12
   */
  public default void setVoltage(double volts) {}

  /** 
   * Commands the hardware to stop. When using TalonFX, this commands the
   * motors to a Neutral control
   */
  public default void stop() {}
}