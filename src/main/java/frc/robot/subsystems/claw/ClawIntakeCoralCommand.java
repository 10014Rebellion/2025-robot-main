package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;

public class ClawIntakeCoralCommand extends Command {
  public final Claw mClawSubsystem;
  public ClawRollerVolt mIntakeVolts;
  public boolean hasOpened;
  public boolean hasReachedBack;

  public ClawIntakeCoralCommand(Claw pClawSubsystem) {
    mClawSubsystem = pClawSubsystem;
    mIntakeVolts = ClawRollerVolt.INTAKE_CORAL;
    hasOpened = false;
    hasReachedBack = false;
  }

  @Override
  public void initialize() {
    mClawSubsystem.setClaw(mIntakeVolts);
  }

  @Override
  public void execute() {
    if (!hasReachedBack) {
      mClawSubsystem.setClaw(mIntakeVolts);
    }

    // if the timer is not running and the claw is open, start the timer
    if (mClawSubsystem.isClawOpen() && !hasOpened) {
      hasOpened = true;
    } else if (mClawSubsystem.getClaw() > 30) {
      mClawSubsystem.setClaw(-mIntakeVolts.get() / 2);
      hasReachedBack = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    mClawSubsystem.setClaw(0);
  }

  @Override
  public boolean isFinished() {
    // if the timer is running and the
    return (hasOpened
        && hasReachedBack
        && (mClawSubsystem.getClaw() > 20 && mClawSubsystem.getClaw() < 23));
  }
}
