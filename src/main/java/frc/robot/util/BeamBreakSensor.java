package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakSensor {
  private final DigitalInput mSensor;

  public BeamBreakSensor(int pDIOPort) {
    mSensor = new DigitalInput(pDIOPort);
  }

  public boolean isBroken() {
    return !mSensor.get();
  }
}
