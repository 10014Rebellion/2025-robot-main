package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawPIDCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.intake.IntakeConstants.IntakePositions;
import frc.robot.subsystems.intake.IntakePIDCommand;
import frc.robot.subsystems.intake.OTBIntake;
import frc.robot.subsystems.intake.autoIntakeCoralCommand;

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(Elevator elevator, Claw claw, OTBIntake intake) {
    addCommands(
        new ParallelDeadlineGroup(
            new autoIntakeCoralCommand(intake),
            new ElevatorPIDCommand(ElevatorConstants.Positions.PREINTAKE, elevator),
            new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, claw)),
        new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, claw),
        new ParallelCommandGroup(
            new IntakePIDCommand(IntakePositions.STOWED, intake),
            new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, elevator),
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(0.75),
                    new InstantCommand(
                        () -> claw.setClaw(ClawConstants.Claw.ClawRollerVolt.INTAKE_CORAL)),
                    new InstantCommand(() -> intake.setFunnel(2))),
                new ParallelCommandGroup(
                    new InstantCommand(() -> claw.setClaw(0)),
                    new InstantCommand(() -> intake.setFunnel(0))))));
  }
}
