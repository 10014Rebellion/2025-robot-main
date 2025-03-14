package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ShootAlgae extends SequentialCommandGroup {
  public ShootAlgae(ClawSubsystem claw) {

    addCommands(
        new ClawPIDCommand(ClawConstants.Wrist.Positions.BARGE, claw),
        new InstantCommand(() -> claw.setClaw(ClawConstants.Claw.ClawRollerVolt.INTAKE_ALGAE)),
        new WaitCommand(1),
        new InstantCommand(() -> claw.setClaw(ClawConstants.Claw.ClawRollerVolt.OUTTAKE_BARGE)));
  }
}
