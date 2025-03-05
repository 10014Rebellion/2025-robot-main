package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;

public class ClawIntakeCoralCommand extends Command {
  public final Claw mClawSubsystem;
  public double mIntakeVolts;
  public boolean hasOpened;

  public ClawIntakeCoralCommand(Claw pClawSubsystem) {
    mClawSubsystem = pClawSubsystem;
    mIntakeVolts = ClawRollerVolt.INTAKE_CORAL.get();
    hasOpened = false;
  }

  @Override
  public void initialize() {
    hasOpened = false;
    // if (mClawSubsystem.getClaw() > 20
    //     && mClawSubsystem.getClaw() < ClawConstants.Claw.ClawOpenPositions.HAS_CORAL.get()) {
    //   hasOpened = true;
    //   mIntakeVolts = 0;
    // }
    mClawSubsystem.setClaw(mIntakeVolts);
  }

  @Override
  public void execute() {
    mClawSubsystem.setClaw(mIntakeVolts);

    // if the timer is not running and the claw is open, start the timer
    if (mClawSubsystem.isClawOpen() && !hasOpened) {
      hasOpened = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    mClawSubsystem.setClaw(0);
  }

  @Override
  public boolean isFinished() {
    // if the timer is running and the
    if (hasOpened
        && (mClawSubsystem.getClaw() < ClawConstants.Claw.ClawOpenPositions.HAS_CORAL.get())) {
      ClawConstants.Claw.hasCoral = true;
    }
    return (ClawConstants.Claw.hasCoral);
  }
}
