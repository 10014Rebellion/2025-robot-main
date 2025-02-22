package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawPIDCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;

public class ExtendOuttake extends ParallelCommandGroup {

  public ExtendOuttake(
      Elevator elevator,
      Claw claw,
      Drive drive,
      ElevatorConstants.Positions elevatorPos,
      ClawConstants.Wrist.Positions clawPos) {
    addCommands(
        new InstantCommand(
            () -> {
              if (elevatorPos.getPos() > 30) {
                drive.setSpeedMultipliers(
                    DriveConstants.kLowSpeedTrans, DriveConstants.kLowSPeedRot);
              } else {
                drive.setSpeedMultipliers(
                    DriveConstants.kHighSpeedTrans, DriveConstants.kHighSpeedRot);
              }
            }),
        new ElevatorPIDCommand(elevatorPos, elevator),
        new SequentialCommandGroup(new WaitCommand(0.5), new ClawPIDCommand(clawPos, claw)));
  }
}
