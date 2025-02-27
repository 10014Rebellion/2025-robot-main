package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawPIDCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class CoralCommands {
    
    public static SequentialCommandGroup scoreCoral(Elevator pElevator, Claw pWrist, 
    Drive pDrive, Vision pVision, ElevatorConstants.Positions elevatorPos, ClawConstants.Wrist.Positions wristPos,
    VisionConstants.PoseOffsets pOffset) {
        // We should change pOffset to be both forward and sideways, not just sideways
        return new SequentialCommandGroup(
            new GoToPose(
                () -> pVision.getReefScoringPose(7, 10, pOffset),
                () -> pVision.getPose(), 
                pDrive),
            new ParallelCommandGroup(
                new ElevatorPIDCommand(elevatorPos, pElevator),
                new SequentialCommandGroup(new WaitCommand(0.25),
                new ClawPIDCommand(wristPos, pWrist))
            ),
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.PREINTAKE, pElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, pWrist)
            )
        );
    }
}
