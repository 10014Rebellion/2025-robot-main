package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawFFCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorFFCommand;

public class RestOuttake extends ParallelCommandGroup {

  public RestOuttake(Elevator elevator, Claw claw) {
    addCommands(new ElevatorFFCommand(elevator), new ClawFFCommand(claw));
  }
}
