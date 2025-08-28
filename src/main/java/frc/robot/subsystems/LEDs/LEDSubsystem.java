package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.LEDs.LEDConstants.ledColor;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED mLED;
  private final AddressableLEDBuffer mLEDBuffer;

  private final int kLEDLength = 36; // TUNE ME

  private ledColor defaultColor = Robot.gIsBlueAlliance ? ledColor.BLUE : ledColor.RED;

  public LEDSubsystem() {

    mLED = new AddressableLED(0);

    mLEDBuffer = new AddressableLEDBuffer(kLEDLength);
    mLED.setLength(mLEDBuffer.getLength());

    setSolid(defaultColor);

    mLED.start();

  }

  public void setDefaultColor() {
    setSolid(defaultColor);
  }

  public void setSolid(ledColor pColor) {
    LEDPattern pattern = LEDPattern.solid(pColor.getColorObj());
    pattern.applyTo(mLEDBuffer);
    mLED.setData(mLEDBuffer);
  }

  @Override
  public void periodic() {
    defaultColor = Robot.gIsBlueAlliance ? ledColor.BLUE : ledColor.RED;
  }
}
