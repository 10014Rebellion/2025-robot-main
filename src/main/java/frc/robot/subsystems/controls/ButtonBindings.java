package frc.robot.subsystems.controls;

import static frc.robot.subsystems.controls.ButtonBindingsConstants.Buttonboard;
import static frc.robot.subsystems.controls.ButtonBindingsConstants.DriverController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.controls.ButtonBindingsConstants.Buttonboard;
import frc.robot.subsystems.controls.ButtonBindingsConstants.DriverController;
import frc.robot.subsystems.controls.TeleopCommands.ActionCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ButtonBindings {
  private final CommandXboxController mDriverController;
  private final CommandGenericHID mOperatorButtonboard;
  private final ActionCommands mActionCommands;

  private final Drive mDrive;
  private final ElevatorSubsystem mElevator;
  private final IntakeSubsystem mIntake;
  private final WristSubsystem mWrist;
  private final ClawSubsystem mClaw;
  private final ClimbSubsystem mClimb;

  public ButtonBindings(Drive pDrive, ElevatorSubsystem pElevator, IntakeSubsystem pIntake, WristSubsystem pWrist, ClawSubsystem pClaw, ClimbSubsystem pClimb){
    this.mDrive = pDrive;
    this.mElevator = pElevator;
    this.mIntake = pIntake;
    this.mWrist = pWrist;
    this.mClaw = pClaw;
    this.mClimb = pClimb;

    this.mDriverController = new CommandXboxController(DriverController.kDriverControllerPort);
    this.mOperatorButtonboard = new CommandGenericHID(Buttonboard.kButtonboardPort);
    this.mActionCommands = new TeleopCommands(pDrive, pWrist, pElevator, pIntake, pClaw, pClimb).new ActionCommands();
  }

  public void initDriverJoysticks() {
    mDrive.acceptJoystickInputs(
			() -> - mDriverController.getLeftY(),
			() -> - mDriverController.getLeftX(),
			() -> mDriverController.getRightX(),
			() -> mDriverController.getHID().getPOV()
    );
  }

  public void initDriverButtons() {
    mDriverController
      .x()
        .onTrue(mActionCommands.getAllianceResetGyroCmd());

    // Menu button, the one with 3 lines like a hamburger menu icon
    mDriverController
      .button(8)
        .onTrue(mActionCommands.getToggleIRCmd());

    mDriverController
      .rightBumper()
        .whileTrue(mActionCommands.getIntakeCoralCmd())
        .whileFalse(mActionCommands.getStowIntakeCmd());
      
    mDriverController
      .leftBumper()
        .whileTrue(mActionCommands.getOuttakeCoralCmd())
        .whileFalse(mActionCommands.getStopIndexerAndRollersCmd());

    // mDriverController
    //   .y()
    //   .whileTrue(
    //       new ParallelCommandGroup(
    //           mWrist
    //               .setPIDCmd(WristConstants.Setpoints.GROUNDALGAE)
    //               .andThen(mWrist.enableFFCmd()),
    //           new ParallelCommandGroup(
    //               mClaw.intakeCoralCmd(),
    //               mElevator.setPIDCmd(
    //                   ElevatorConstants.Setpoints.GROUNDALGAE))))
    //   .whileFalse(
    //       new ParallelCommandGroup(
    //           new InstantCommand(() -> currentScoreLevel = 0),
    //           mElevator.setPIDCmd(
    //               ElevatorConstants.Setpoints.HOLD_ALGAE),
    //           mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE)
    //               .andThen(mWrist.enableFFCmd()),
    //           mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE
    //               .get())));


    mDriverController
      .povUp()
      .whileTrue(mClimb.setGrabberVoltsCmd(ClimbConstants.Grabber.VoltageSetpoints.PULL_IN));

    mDriverController.povDown().whileTrue(mClimb.retractClimb());
  }

  
	public void initDriverController() {
    
	}
}
