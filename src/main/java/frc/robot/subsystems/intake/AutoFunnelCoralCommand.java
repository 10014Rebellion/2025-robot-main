package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoFunnelCoralCommand extends Command {
  private final IntakeSubsystem mIntakeSubsystem;

  public AutoFunnelCoralCommand(IntakeSubsystem pIntakeSubsystem) {
    mIntakeSubsystem = pIntakeSubsystem;
  }

  @Override
  public void execute() {
    // If coral is not in indexer we set to high speed
    if (!mIntakeSubsystem.getCoralDetectedBack() && !mIntakeSubsystem.getCoralDetectedFront()) { 
      mIntakeSubsystem.setVoltsIndexer(3); // TODO: Tune this
    } 
    // If coral is past the first sensor but not at second sensor we set to medium speed
    else if (mIntakeSubsystem.getCoralDetectedBack() && !mIntakeSubsystem.getCoralDetectedFront()) {
      mIntakeSubsystem.setVoltsIndexer(2); // TODO: Tune this
    }
  }

  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.setVoltsIndexer(0);
  }

  @Override
  public boolean isFinished() {
    return mIntakeSubsystem.getCoralDetected();
  }
}
