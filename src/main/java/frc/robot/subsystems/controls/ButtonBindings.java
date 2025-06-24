package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDs.LEDConstants.ledColor;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.controls.ButtonBindingsConstants.Buttonboard;
import frc.robot.subsystems.controls.ButtonBindingsConstants.DriverController;
import frc.robot.subsystems.controls.StateTracker.CoralLevel;
import frc.robot.subsystems.controls.TeleopCommands.ActionCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser.SIDE;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
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
  private final LEDSubsystem mLEDs;

  public ButtonBindings(Drive pDrive, ElevatorSubsystem pElevator, IntakeSubsystem pIntake, WristSubsystem pWrist, ClawSubsystem pClaw, ClimbSubsystem pClimb, LEDSubsystem pLEDs){
    this.mDrive = pDrive;
    this.mElevator = pElevator;
    this.mIntake = pIntake;
    this.mWrist = pWrist;
    this.mClaw = pClaw;
    this.mClimb = pClimb;
    this.mLEDs = pLEDs;

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

  public void initTriggers() {
    new Trigger(
      () -> (mDrive.atGoal() && mElevator.isPIDAtGoal() && mWrist.isPIDAtGoal()))
        .whileTrue(mActionCommands.getScoreCoralCmd());

      new Trigger(() -> mClaw.getBeamBreak())
        .whileTrue(new InstantCommand(() -> mLEDs.setSolid(ledColor.YELLOW)))
        .whileFalse(new InstantCommand(() -> mLEDs.setDefaultColor()));

    new Trigger(() -> mDrive.atGoal())
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> mLEDs.setSolid(ledColor.GREEN))))
        .whileFalse(new InstantCommand(() -> mLEDs.setDefaultColor()));
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
    
    mDriverController
      .leftTrigger()
        .whileTrue(mActionCommands.getGoToReefCmd(SIDE.LEFT))
        .onFalse(mActionCommands.getStopDriveCmd());

    mDriverController
      .rightTrigger()
        .whileTrue(mActionCommands.getGoToReefCmd(SIDE.RIGHT))
        .onFalse(mActionCommands.getStopDriveCmd());

    mDriverController
      .a()
        .whileTrue(mActionCommands.getGoToReefCmd(SIDE.ALGAE))
        .onFalse(mActionCommands.getStopDriveCmd());

    mDriverController
      .y()
        .whileTrue(mActionCommands.getGroundAlgaeCmd())
        .whileFalse(mActionCommands.getHoldAlgaeCmd());

    mDriverController
      .povUp()
        .whileTrue(mClimb.setGrabberVoltsCmd(ClimbConstants.Grabber.VoltageSetpoints.PULL_IN));

    mDriverController.povDown().whileTrue(mClimb.retractClimb());
  }

  public void initOperatorButtons() {
    mOperatorButtonboard
      .button(ButtonBindingsConstants.Buttonboard.kScoreCoral)
        .whileTrue(mActionCommands.getScoreCoralCmd());

    mOperatorButtonboard
      .button(ButtonBindingsConstants.Buttonboard.kClimbAscend)
        .whileTrue(
            new ParallelCommandGroup(
                mWrist.setPIDCmd(WristConstants.Setpoints.CLIMB).andThen(mWrist.enableFFCmd()),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.Climb),
                mClimb.climbToSetpoint(ClimbConstants.Pulley.Setpoints.CLIMBED),
                mClimb.setGrabberVoltsCmd(0.0),
                mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.STOWED)));

    mOperatorButtonboard
      .button(ButtonBindingsConstants.Buttonboard.kClimbDescend)
        .whileTrue(mClimb.climbToSetpoint(ClimbConstants.Pulley.Setpoints.EXTENDED));
    // .whileFalse();

    mOperatorButtonboard
      .button(ButtonBindingsConstants.Buttonboard.kSetScoreL4)
        .whileTrue(mActionCommands.getPrepCoralScoreCmd(CoralLevel.B3));

    mOperatorButtonboard
      .button(ButtonBindingsConstants.Buttonboard.kSetScoreL3)
        .whileTrue(mActionCommands.getPrepCoralScoreCmd(CoralLevel.B2));

    mOperatorButtonboard
      .button(ButtonBindingsConstants.Buttonboard.kSetScoreL2)
        .whileTrue(mActionCommands.getPrepCoralScoreCmd(CoralLevel.B1));

    mOperatorButtonboard
        .button(ButtonBindingsConstants.Buttonboard.kSetScoreL1)
          .whileTrue(mActionCommands.getPrepCoralScoreCmd(CoralLevel.T));

    mOperatorButtonboard
        .button(ButtonBindingsConstants.Buttonboard.kAlgaePickupL2)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.L2ALGAE).andThen(mWrist.enableFFCmd()),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get())))
        .onFalse(mActionCommands.getHoldAlgaeCmd());

    mOperatorButtonboard
        .button(ButtonBindingsConstants.Buttonboard.kAlgaePickupL3)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.L3ALGAE).andThen(mWrist.enableFFCmd()),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get())))
        .onFalse(mActionCommands.getHoldAlgaeCmd());

    mOperatorButtonboard
        .button(ButtonBindingsConstants.Buttonboard.kPickup)
        .whileTrue(
            new SequentialCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE),
                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE),
                new ParallelCommandGroup(
                    mClaw.intakeCoralCmd(),
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.POSTINTAKE))))
        .whileFalse(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE),
                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE)));

    mOperatorButtonboard.axisLessThan(1, -0.5).whileTrue(mElevator.setVoltsCmd(5));

    mOperatorButtonboard.axisGreaterThan(1, 0.5).whileTrue(mElevator.setVoltsCmd(-3));

    mOperatorButtonboard.axisGreaterThan(0, 0.5).whileTrue(mWrist.setVoltsCmd(1.5));

    mOperatorButtonboard.axisLessThan(0, -0.50).whileTrue(mWrist.setVoltsCmd(-1.5));

    mOperatorButtonboard
      .button(ButtonBindingsConstants.Buttonboard.kEjectAlgaeToBarge)
        .whileTrue(mClaw.setClawCmd(ClawConstants.RollerSpeed.OUTTAKE_L1.get()))
        .whileFalse(new InstantCommand(() -> mClaw.setClaw(0)));

    mOperatorButtonboard
      .button(ButtonBindingsConstants.Buttonboard.kGoToBarge)
        .whileTrue(mActionCommands.getScoreBargeCmd());
  }
}
