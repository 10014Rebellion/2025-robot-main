package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawLevelPIDCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorLevelPIDCommand;

public class ManipulatorLevelCommand extends ParallelCommandGroup {

  public ManipulatorLevelCommand(Elevator elevator, Claw claw, Drive drive) {
    addCommands(
        new InstantCommand(() -> drive.updateSpeedMultipliers()),
        new SequentialCommandGroup(
            new ElevatorLevelPIDCommand(elevator), new ClawLevelPIDCommand(claw)));
  }
}
