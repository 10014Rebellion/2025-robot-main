package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoFunnelCoralCommand extends Command {
  private final IntakeSubsystem mIntakeSubsystem;
  private boolean mBackTriggered;

  public AutoFunnelCoralCommand(IntakeSubsystem pIntakeSubsystem) {
    mIntakeSubsystem = pIntakeSubsystem;
    mBackTriggered = false;
  }

  @Override
  public void execute() {
    if (mIntakeSubsystem.getCoralDetectedBack()) {
      mBackTriggered = true;
    }

    if (mBackTriggered) {
      mIntakeSubsystem.setVoltsIndexer(IntakeConstants.Indexer.kIntakeVoltsSlow);
    } else {
      mIntakeSubsystem.setVoltsIndexer(IntakeConstants.Indexer.kIntakeVolts);
    }
  }

  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.setVoltsIndexer(1);
  }

  @Override
  public boolean isFinished() {
    return mIntakeSubsystem.getCoralDetected();
  }
}
