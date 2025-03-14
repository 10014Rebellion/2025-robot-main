package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorFFCommand;

public class RestOuttake extends ParallelCommandGroup {

  public RestOuttake(ElevatorSubsystem elevator, ClawSubsystem claw) {
    addCommands(new ElevatorFFCommand(elevator), new ClawFFCommand(claw));
  }
}
