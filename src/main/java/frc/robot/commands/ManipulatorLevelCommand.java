package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorLevelPIDCommand;

public class ManipulatorLevelCommand extends ParallelCommandGroup {

  public ManipulatorLevelCommand(ElevatorSubsystem elevator, ClawSubsystem claw, DriveSubsystem drive) {
    addCommands(
        new InstantCommand(() -> drive.updateSpeedMultipliers()),
        new SequentialCommandGroup(
            new ElevatorLevelPIDCommand(elevator), new ClawLevelPIDCommand(claw)));
  }
}
