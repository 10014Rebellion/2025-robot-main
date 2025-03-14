package frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewLEDInterface extends SubsystemBase{

    private final int LEDLength = 100;

    AddressableLED mLED;
    AddressableLEDBuffer mLEDBuffer;

    // ---------CONSTRUCTOR(S)----------------\
      /**
       * LEDSubsystem
       * @param port PWM port on the roboRIO
       */
    public NewLEDInterface(int port) {
        mLED = new AddressableLED(port);
        mLED.setLength(LEDLength);
        mLEDBuffer = new AddressableLEDBuffer(LEDLength);
    }
}
