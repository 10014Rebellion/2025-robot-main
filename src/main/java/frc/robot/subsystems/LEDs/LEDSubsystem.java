package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.LEDs.LEDConstants.ledColor;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED mLED;
  private final AddressableLEDBuffer mLEDBuffer;

  private final int kLEDLength = 30; // TUNE ME

  private final ledColor defaultColor = Robot.gIsBlueAlliance ? ledColor.BLUE : ledColor.RED;

  public LEDSubsystem(
      ClawSubsystem clawSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      DriveSubsystem driveSubsystem) {

    mLED = new AddressableLED(0);

    mLEDBuffer = new AddressableLEDBuffer(kLEDLength);
    mLED.setLength(mLEDBuffer.getLength());

    setSolid(defaultColor);

    mLED.start();

    initTriggers(clawSubsystem, intakeSubsystem, elevatorSubsystem, wristSubsystem, driveSubsystem);
  }

  private void setSolid(ledColor pColor) {
    LEDPattern pattern = LEDPattern.solid(pColor.getColorObj());
    pattern.applyTo(mLEDBuffer);
    mLED.setData(mLEDBuffer);
  }

  // private void setPattern(ledPatterns pPattern) {
  //   pPattern.getPattern().applyTo(mLEDBuffer);
  //   mLED.setData(mLEDBuffer);
  // }

  private void initTriggers(
      ClawSubsystem clawSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      DriveSubsystem driveSubsystem) {
    new Trigger(() -> clawSubsystem.getBeamBreak())
        .whileTrue(new InstantCommand(() -> setSolid(ledColor.YELLOW)))
        .whileFalse(new InstantCommand(() -> setSolid(defaultColor)));

    new Trigger(() -> driveSubsystem.isAtPose)
        // && elevatorSubsystem.isPIDAtGoal()
        // // && clawSubsystem.getBeamBreak()
        // && wristSubsystem.isPIDAtGoal())
        .whileTrue(new InstantCommand(() -> setSolid(ledColor.GREEN)))
        .whileFalse(new InstantCommand(() -> setSolid(defaultColor)));
  }

  @Override
  public void periodic() {}
}
