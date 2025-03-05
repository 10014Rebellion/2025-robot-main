package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;

public class ClawIntakeCoralCommand extends Command {
  public final Claw mClawSubsystem;
  public ClawRollerVolt mIntakeVolts;
  public Timer mIntakeTime;

  public ClawIntakeCoralCommand(Claw pClawSubsystem) {
    mClawSubsystem = pClawSubsystem;
    mIntakeVolts = ClawRollerVolt.INTAKE_CORAL;
    mIntakeTime = new Timer();
  }

  @Override
  public void initialize() {
    mClawSubsystem.setClaw(mIntakeVolts);
    mIntakeTime.stop();
  }

  @Override
  public void execute() {
    mClawSubsystem.setClaw(mIntakeVolts);
  }

  @Override
  public void end(boolean interrupted) {
    mClawSubsystem.setClaw(0);
  }

  @Override
  public boolean isFinished() {
    // if the timer is running and the
    return (mIntakeTime.get() > 2);
  }
}
