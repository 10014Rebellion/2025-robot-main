package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class elevatorManualCommand extends Command {
  private final Elevator mElevator;
  private double mVolts;

  public elevatorManualCommand(Elevator elevatorSubsystem, double setVolts) {
    this.mElevator = elevatorSubsystem;
    // for some reason + input = - output which is weird
    this.mVolts = -setVolts;
    addRequirements(mElevator);
  }

  @Override
  public void initialize() {
    mElevator.setMotorVoltage(mVolts);
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mElevator.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    mElevator.setMotorVoltage(mVolts);
  }

  @Override
  public void end(boolean interrupted) {
    mElevator.setMotorVoltage(0);
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mElevator.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
