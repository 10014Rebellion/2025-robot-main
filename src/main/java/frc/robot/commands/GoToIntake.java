package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawIntakeCoralCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class GoToIntake extends ParallelCommandGroup {
  public GoToIntake(ElevatorSubsystem pElevator, ClawSubsystem pClaw, IntakeSubsystem pIntake) {
    addCommands(
        new SequentialCommandGroup(
            new ReadyForIntake(elevator, claw),
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, elevator),
                new ClawIntakeCoralCommand(claw)
                )));
  }
}
