package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorLevelPIDCommand;
import frc.robot.subsystems.claw.ClawLevelPIDCommand;

public class ManipulatorLevelCommand extends ParallelCommandGroup {

  public ManipulatorLevelCommand(
      Elevator elevator,
      Claw claw,
      Drive drive) {
    addCommands(
        new InstantCommand(
            () -> {
              if (SmartDashboard.getNumber("Levels/Elevator Setpoint", 0) >= 40) {
                drive.setSpeedMultipliers(
                    DriveConstants.kLowSpeedTrans, DriveConstants.kLowSPeedRot);
              } else {
                drive.setSpeedMultipliers(
                    DriveConstants.kHighSpeedTrans, DriveConstants.kHighSpeedRot);
              }
            }),
        new SequentialCommandGroup(
          new ElevatorLevelPIDCommand(elevator),
          new ClawLevelPIDCommand(claw)
        ));
  }
}
