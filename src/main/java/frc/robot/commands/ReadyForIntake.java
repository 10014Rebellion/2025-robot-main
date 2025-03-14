package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;

public class ReadyForIntake extends ParallelCommandGroup {
  public ReadyForIntake(ElevatorSubsystem elevator, ClawSubsystem claw) {
    addCommands(
        new ParallelCommandGroup(
            new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, claw),
            new ElevatorPIDCommand(ElevatorConstants.Positions.PREINTAKE, elevator)));
  }
}
