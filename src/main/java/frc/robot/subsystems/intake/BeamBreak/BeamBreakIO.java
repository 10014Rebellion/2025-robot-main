
package frc.robot.subsystems.intake.BeamBreak;

import org.littletonrobotics.junction.AutoLog;

/** The intake subsystem's hardware interface. */
public interface BeamBreakIO {
  @AutoLog
  public static class BeamBreakIOInputs {
    public boolean isSensorDetected = true;
    public boolean isDetected = true;
  }

  /**
   * Write data from the hardware to the inputs object
   * 
   * @param inputs The inputs object
   */
  public default void updateInputs(BeamBreakIOInputs inputs) {};

  public default boolean isDetected() {return false;}

}