
package frc.robot.subsystems.claw.sensor;

import org.littletonrobotics.junction.AutoLog;

/** The intake subsystem's hardware interface. */
public interface SensorIO {
  @AutoLog
  public static class SensorIOInputs {
    public boolean isSensorConnected = false;

    public double distanceFromClaw = 0.0;
    public double distanceFromClawStddev = 0.0;
    public boolean isCANRangeDetected = false;
    public boolean isBeamBreakDetected = false;
    public double ambience = 0.0;
    public double signalStrength = 0.0;
    public boolean isMeasurementHealthGood = false;
    public double measurementTime = 0.0;
  }

  /**
   * Write data from the hardware to the inputs object
   * 
   * @param inputs The inputs object
   */
  public default void updateInputs(SensorIOInputs inputs) {};

  public default boolean getBeamBreakValue() {return false;};

  public default boolean getCANRangeValue() { return false;};

}