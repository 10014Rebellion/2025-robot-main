package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoFunnelCoralCommand extends Command {
  private final IntakeSubsystem mIntakeSubsystem;

  public AutoFunnelCoralCommand(IntakeSubsystem pIntakeSubsystem) {
    mIntakeSubsystem = pIntakeSubsystem;
  }

  @Override
  public void execute() {
    mIntakeSubsystem.setVoltsIndexer(IntakeConstants.Indexer.kIntakeVolts);
  }

  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.setVoltsIndexer(0.1);
  }

  @Override
  public boolean isFinished() {
    return mIntakeSubsystem.getCoralDetected();
  }
}
