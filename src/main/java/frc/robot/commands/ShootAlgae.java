package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawPIDCommand;

public class ShootAlgae extends SequentialCommandGroup {
  public ShootAlgae(Claw claw) {

    addCommands(
        new ClawPIDCommand(ClawConstants.Wrist.Positions.BARGE, claw),
        new InstantCommand(() -> claw.setClaw(ClawConstants.Claw.ClawRollerVolt.INTAKE_ALGAE)),
        new WaitCommand(1),
        new InstantCommand(() -> claw.setClaw(ClawConstants.Claw.ClawRollerVolt.OUTTAKE_BARGE)));
  }
}
