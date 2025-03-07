package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.Command;

public class ClawManualCommand extends Command {
  private final Claw mClaw;
  private double mVolts;

  public ClawManualCommand(Claw clawSubsystem, double setVolts) {
    this.mClaw = clawSubsystem;
    this.mVolts = setVolts;
    addRequirements(mClaw);
  }

  @Override
  public void initialize() {
    mClaw.setWrist(mVolts);
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mClaw.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    mClaw.setWrist(mVolts);
  }

  @Override
  public void end(boolean interrupted) {
    mClaw.setWrist(0);
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mClaw.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
