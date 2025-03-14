package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Setpoints;

public class ClawIntakeCoralCommand extends Command {
  public final ClawSubsystem mClawSubsystem;
  public double mIntakeVolts;
  private boolean hasOpened;
  private boolean hasReachedBack;

  public ClawIntakeCoralCommand(ClawSubsystem pClawSubsystem) {
    mClawSubsystem = pClawSubsystem;
    mIntakeVolts = Setpoints.INTAKE_CORAL.get();
    hasOpened = false;
    hasReachedBack = false;
  }

  @Override
  public void initialize() {
    hasOpened = false;
    hasReachedBack = false;
    mIntakeVolts = Setpoints.INTAKE_CORAL.get();
    // if (mClawSubsystem.getClaw() > 20
    //     && mClawSubsystem.getClaw() < ClawConstants.Claw.ClawOpenPositions.HAS_CORAL.get()) {
    //   hasOpened = true;
    //   mIntakeVolts = 0;
    // }
    mClawSubsystem.setClaw(mIntakeVolts);
    SmartDashboard.putBoolean("Has Opened", hasOpened);
  }

  @Override
  public void execute() {
    mClawSubsystem.setClaw(mIntakeVolts);

    if (mClawSubsystem.isClawOpen() && !hasOpened) {
      hasOpened = true;
    }

    if (mClawSubsystem.getClaw() > ClawConstants.ClawOpenPositions.MAX.get()
        && !hasReachedBack) {
      hasReachedBack = true;
      mIntakeVolts = -mIntakeVolts / 2;
    }

    SmartDashboard.putBoolean("Has Opened", hasOpened);
  }

  @Override
  public void end(boolean interrupted) {
    mClawSubsystem.setClaw(0);
    // if (mClawSubsystem.getClaw() < ClawConstants.Claw.ClawOpenPositions.NO_CORAL.get()) {
    //   // basically, if the claw isn't open enough to have a coral,
    //   // we assume that there isn't actually a coral in the claw
    //   ClawConstants.Claw.hasCoral = false;
    // }
  }

  @Override
  public boolean isFinished() {
    return hasOpened && mClawSubsystem.hasCoral();
  }
}
