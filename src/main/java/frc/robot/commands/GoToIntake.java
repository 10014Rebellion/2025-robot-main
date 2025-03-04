package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIntakeCoralCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.intake.OTBIntake;

public class GoToIntake extends ParallelCommandGroup {
  public GoToIntake(Elevator elevator, Claw claw, OTBIntake intake) {
    addCommands(
        new SequentialCommandGroup(
            new ReadyForIntake(elevator, claw),
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, elevator),
                new ClawIntakeCoralCommand(claw)
                // new SequentialCommandGroup(
                //     new ParallelDeadlineGroup(
                //         new WaitCommand(0.75),
                //         new InstantCommand(
                //             () -> claw.setClaw(ClawConstants.Claw.ClawRollerVolt.INTAKE_CORAL)),
                //         new InstantCommand(() -> intake.setFunnel(2))),
                //     new ParallelCommandGroup(
                //         new InstantCommand(() -> claw.setClaw(0)),
                //         new InstantCommand(() -> intake.setFunnel(0))
                //     )
                // )
                )));
  }
}
