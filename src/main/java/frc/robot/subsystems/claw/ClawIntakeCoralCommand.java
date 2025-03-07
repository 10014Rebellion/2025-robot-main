package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;

public class ClawIntakeCoralCommand extends Command {
  public final Claw mClawSubsystem;
  public double mIntakeVolts;
  private boolean hasOpened;
  private boolean hasReachedBack;

  public ClawIntakeCoralCommand(Claw pClawSubsystem) {
    mClawSubsystem = pClawSubsystem;
    mIntakeVolts = ClawRollerVolt.INTAKE_CORAL.get();
    hasOpened = false;
    hasReachedBack = false;
  }

  @Override
  public void initialize() {
    hasOpened = false;
    hasReachedBack = false;
    mIntakeVolts = ClawRollerVolt.INTAKE_CORAL.get();
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

    if (mClawSubsystem.getClaw() > ClawConstants.Claw.ClawOpenPositions.MAX.get()
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
    if (hasOpened
        && (mClawSubsystem.getClaw() < ClawConstants.Claw.ClawOpenPositions.HAS_CORAL.get())) {
      ClawConstants.Claw.hasCoral = true;
    }
    return (ClawConstants.Claw.hasCoral);
  }
}
