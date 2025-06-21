// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.controls.TeleopCommands.ControllerCommands;
import frc.robot.subsystems.controls.TeleopCommands.ControllerCommands.ControllerTuningCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionConstants.PoseOffsets;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.commands.DynamicCommand;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ControlsSubsystem extends SubsystemBase {
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandGenericHID operatorButtonboard = new CommandGenericHID(1);
	private final ControllerCommands controllerCommands;
	private final ControllerTuningCommands controllerTUNING;


	private int currentScoreLevel;
	private Supplier<VisionConstants.PoseOffsets> sideScoring;
	private boolean mSwerveFieldOriented = true;
	private boolean doAutoScore = true;
	private boolean goingToBarge = false;

	private final DriveSubsystem mDrive;
	private final VisionSubsystem mVision;
	private final WristSubsystem mWrist;
	private final ElevatorSubsystem mElevator;
	private final IntakeSubsystem mIntake;
	private final ClawSubsystem mClaw;
	private final ClimbSubsystem mClimb;
	private boolean isAligningToBarge;

	public ControlsSubsystem(
			DriveSubsystem pDrive,
			VisionSubsystem pVision,
			WristSubsystem pWrist,
			ElevatorSubsystem pElevator,
			IntakeSubsystem pIntake,
			ClawSubsystem pClaw,
			ClimbSubsystem pClimb) {
		this.mDrive = pDrive;
		this.mVision = pVision;
		this.mWrist = pWrist;
		this.mElevator = pElevator;
		this.mIntake = pIntake;
		this.mClaw = pClaw;
		this.mClimb = pClimb;
		TeleopCommands teleopCommands = new TeleopCommands(pDrive, pVision, pWrist, pElevator, pIntake, pClaw,
				pClimb);
		controllerCommands = teleopCommands.new ControllerCommands();
		controllerTUNING = controllerCommands.new ControllerTuningCommands();

		isAligningToBarge = false;
		currentScoreLevel = 4;
		sideScoring = () -> VisionConstants.PoseOffsets.LEFT;
		SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Setpoints.L2.getPos());
		SmartDashboard.putNumber("Levels/Wrist Setpoint", WristConstants.Setpoints.L2.getPos());
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean(
				"Levels/Left Side Chosen", sideScoring.get().equals(VisionConstants.PoseOffsets.LEFT));
		SmartDashboard.putNumber("Levels/Current Coral Level", currentScoreLevel);
		SmartDashboard.putBoolean("Levels/GOTOBARGE", goingToBarge);
		SmartDashboard.putBoolean("Levels/Auto Score", doAutoScore);
	}

	public void initTriggers() {
		new Trigger(
				() -> (mDrive.isAtPose && mElevator.isPIDAtGoal() && mWrist.isPIDAtGoal()
						&& doAutoScore))
				.whileTrue(new DynamicCommand(() -> getScoreCmd(currentScoreLevel)));
	}

	public void initDriverController() {
		driverController
			.rightBumper()
				.whileTrue(controllerCommands.intakeCoral)
				.whileFalse(controllerCommands.stowIntake);
				
		driverController
				.leftBumper()
				.whileTrue(controllerCommands.outtakeCoral)
				.whileFalse(controllerCommands.stopIndexerAndRollers);

		driverController
				.y()
				.whileTrue(
						new ParallelCommandGroup(
								mWrist
										.setPIDCmd(WristConstants.Setpoints.GROUNDALGAE)
										.andThen(mWrist.enableFFCmd()),
								new ParallelCommandGroup(
										mClaw.intakeCoralCmd(),
										mElevator.setPIDCmd(
												ElevatorConstants.Setpoints.GROUNDALGAE))))
				.whileFalse(
						new ParallelCommandGroup(
								new InstantCommand(() -> currentScoreLevel = 0),
								mElevator.setPIDCmd(
										ElevatorConstants.Setpoints.HOLD_ALGAE),
								mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE)
										.andThen(mWrist.enableFFCmd()),
								mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE
										.get())));

		driverController.povRight().onTrue(new InstantCommand(() -> doAutoScore = !doAutoScore));

		driverController
				.povUp()
				.whileTrue(mClimb.setGrabberVoltsCmd(ClimbConstants.Grabber.VoltageSetpoints.PULL_IN));
		driverController.povDown().whileTrue(mClimb.retractClimb());
	}

	public void initDrivebase() {
		mDrive.setDefaultCommand(controllerCommands.getDriveCmd(driverController, mSwerveFieldOriented, isAligningToBarge));

		driverController
				.b()
				.onTrue(new InstantCommand(() -> mSwerveFieldOriented = !mSwerveFieldOriented));

		driverController
				.rightTrigger()
				.whileTrue(controllerCommands.getGoToPoseCmd(PoseOffsets.RIGHT));

		driverController
				.leftTrigger()
				.whileTrue(controllerCommands.getGoToPoseCmd(PoseOffsets.LEFT));

		driverController
				.a()
				.whileTrue(controllerCommands.getGoToPoseCmd(PoseOffsets.CENTER));

		driverController
				.x()
				.onTrue(controllerCommands.allianceResetGyro);

		// Menu button, the one with 3 lines like a hamburger menu icon
		driverController.button(8).onTrue(new InstantCommand(() -> mIntake.toggleIRSensor()));

		// Weird button with the two rectangles inside each other, idk what its called
		// driverController.button(7).onTrue(new InstantCommand(() ->
		// mIntake.toggleIRSensor()));
	}

	public void initTuningDrive() {
		mDrive.setDefaultCommand(controllerCommands.getDriveCmd(driverController, mSwerveFieldOriented));

		driverController
				.b()
				.onTrue(new InstantCommand(() -> mSwerveFieldOriented = !mSwerveFieldOriented));

		driverController
				.x()
				.onTrue(controllerTUNING.resetGyro);

		driverController
				.povUp()
				.whileTrue(
						DriveCommands.joystickDrive(
								mDrive, () -> -1, () -> 0, () -> 0,
								() -> mSwerveFieldOriented));

		driverController
				.povDown()
				.whileTrue(
						DriveCommands.joystickDrive(
								mDrive, () -> 1, () -> 0, () -> 0,
								() -> mSwerveFieldOriented));

		driverController
				.povLeft()
				.whileTrue(
						DriveCommands.joystickDrive(
								mDrive, () -> 0, () -> 0, () -> 1,
								() -> mSwerveFieldOriented));

		driverController
				.povRight()
				.whileTrue(
						DriveCommands.joystickDrive(
								mDrive, () -> 0, () -> 0, () -> -1,
								() -> mSwerveFieldOriented));

		driverController
				.a()
				.whileTrue(
						new GoToPose(
								() -> {
									Pose2d visionPose = mVision
											.getPoseInFrontOfAprilTag(21,
													16);
									Pose2d newPose = new Pose2d(
											visionPose.getTranslation(),
											new Rotation2d(Units
													.degreesToRadians(
															45)));
									Logger.recordOutput("Debugging/Tuning Setpoint",
											newPose);
									return newPose;
								},
								() -> mDrive.getPose(),
								mDrive));
	}

	public void initOperatorButtonboard() {

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kScoreCoral)
				.whileTrue(new DynamicCommand(() -> getScoreCmd(currentScoreLevel)));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kClimbAscend)
				.whileTrue(
						new ParallelCommandGroup(
								mWrist.setPIDCmd(WristConstants.Setpoints.CLIMB)
										.andThen(mWrist.enableFFCmd()),
								mElevator.setPIDCmd(ElevatorConstants.Setpoints.Climb),
								mClimb.climbToSetpoint(
										ClimbConstants.Pulley.Setpoints.CLIMBED),
								mClimb.setGrabberVoltsCmd(0.0),
								mIntake.setPIDIntakePivotCmd(
										IntakeConstants.IntakePivot.Setpoints.STOWED)));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kClimbDescend)
				.whileTrue(mClimb.climbToSetpoint(ClimbConstants.Pulley.Setpoints.EXTENDED));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kSetScoreL4)
				.whileTrue(
						new ParallelCommandGroup(
								new InstantCommand(() -> currentScoreLevel = 4),
								mElevator.setPIDCmd(ElevatorConstants.Setpoints.L4),
								mWrist.setPIDCmd(WristConstants.Setpoints.L4)));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kSetScoreL3)
				.whileTrue(
						new ParallelCommandGroup(
								new InstantCommand(() -> currentScoreLevel = 3),
								mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3),
								mWrist.setPIDCmd(WristConstants.Setpoints.L3)));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kSetScoreL2)
				.whileTrue(
						new SequentialCommandGroup(
								new InstantCommand(() -> currentScoreLevel = 2),
								mWrist.setPIDCmd(WristConstants.Setpoints.L2),
								mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2)));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kSetScoreL1)
				.whileTrue(
						new ParallelCommandGroup(
								new InstantCommand(() -> currentScoreLevel = 1),
								mElevator.setPIDCmd(ElevatorConstants.Setpoints.L1),
								mWrist.setPIDCmd(WristConstants.Setpoints.L1)));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kAlgaePickupL2)
				.whileTrue(
						new ParallelCommandGroup(
								mElevator.setPIDCmd(
										ElevatorConstants.Setpoints.L2ALGAE),
								mWrist.setPIDCmd(WristConstants.Setpoints.L2ALGAE)
										.andThen(mWrist.enableFFCmd()),
								mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE
										.get())))
				// .onFalse(mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get()))
				.onFalse(
						new ParallelCommandGroup(
								new InstantCommand(() -> currentScoreLevel = 0),
								mElevator.setPIDCmd(
										ElevatorConstants.Setpoints.HOLD_ALGAE),
								mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE)
										.andThen(mWrist.enableFFCmd()),
								mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE
										.get())));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kAlgaePickupL3)
				.whileTrue(
						new ParallelCommandGroup(
								mElevator.setPIDCmd(
										ElevatorConstants.Setpoints.L3ALGAE),
								mWrist.setPIDCmd(WristConstants.Setpoints.L3ALGAE)
										.andThen(mWrist.enableFFCmd()),
								mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE
										.get())))
				.onFalse(
						new ParallelCommandGroup(
								new InstantCommand(() -> currentScoreLevel = 0),
								mElevator.setPIDCmd(
										ElevatorConstants.Setpoints.HOLD_ALGAE),
								mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE)
										.andThen(mWrist.enableFFCmd()),
								mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE
										.get())));
		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kPickup)
				.whileTrue(
						new SequentialCommandGroup(
								mElevator.setPIDCmd(
										ElevatorConstants.Setpoints.PREINTAKE),
								mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE),
								new ParallelCommandGroup(
										mClaw.intakeCoralCmd(),
										mElevator.setPIDCmd(
												ElevatorConstants.Setpoints.POSTINTAKE))))
				.whileFalse(
						new ParallelCommandGroup(
								mElevator.setPIDCmd(
										ElevatorConstants.Setpoints.PREINTAKE),
								mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE)));

		operatorButtonboard.axisLessThan(1, -0.5).whileTrue(mElevator.setVoltsCmd(5));

		operatorButtonboard.axisGreaterThan(1, 0.5).whileTrue(mElevator.setVoltsCmd(-3));

		operatorButtonboard.axisGreaterThan(0, 0.5).whileTrue(mWrist.setVoltsCmd(1.5));

		operatorButtonboard.axisLessThan(0, -0.50).whileTrue(mWrist.setVoltsCmd(-1.5));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kEjectAlgaeToBarge)
				.whileTrue(mClaw.setClawCmd(ClawConstants.RollerSpeed.OUTTAKE_L1.get()))
				.whileFalse(new InstantCommand(() -> mClaw.setClaw(0)));

		operatorButtonboard
				.button(ControlsConstants.Buttonboard.kGoToBarge)
				.whileTrue(
						new ParallelCommandGroup(
								new InstantCommand(() -> currentScoreLevel = 5),
								new DynamicCommand(
										() -> getScoreCmd(
												5))))
				.whileFalse(new InstantCommand(() -> goingToBarge = false));
	}

	public void initIntakeTuning() {
		driverController
				.povLeft()
				.whileTrue(
						mIntake.setTunablePIDIntakeCommand(
								IntakeConstants.IntakePivot.Setpoints.STOWED.getPos()));
		driverController
				.povRight()
				.whileTrue(
						mIntake.setTunablePIDIntakeCommand(
								IntakeConstants.IntakePivot.Setpoints.INTAKING
										.getPos()));

		driverController.povUp().whileTrue(mIntake.setPivotCmd(3.0));
		driverController.povDown().whileTrue(mIntake.setPivotCmd(-3));

		driverController.rightBumper().whileTrue(mIntake.setRollerCmd(6.0));
		driverController.leftBumper().whileTrue(mIntake.setRollerCmd(-6.0));
	}

	public void initWristTuning() {
		driverController.povUp().whileTrue(mWrist.setTunablePIDCmd(90.0));
		driverController.povRight().whileTrue(mWrist.setTunablePIDCmd(45.0));
		driverController.povDown().whileTrue(mWrist.setTunablePIDCmd(0.0));
		driverController
				.povLeft()
				.whileTrue(
						mElevator
								.setPIDCmd(ElevatorConstants.Setpoints.L2)
								.andThen(mWrist.setTunablePIDCmd(-30.0)));
	}

	public void initElevatorTuning() {
		driverController.povUp().whileTrue(mElevator.setTunablePIDCommand());
		driverController.x().whileTrue(mElevator.setVoltsCmd(-2));
		driverController.b().whileTrue(mElevator.setVoltsCmd(2));
	}

	private Command getScoreCmd(int level) {
		int curLevel = MathUtil.clamp(level, 0, 5);

		if (curLevel == 1) {
			return mClaw.setClawCmd(ClawConstants.RollerSpeed.OUTTAKE_L1.get());
		} else if (curLevel == 2) {
			return new ParallelCommandGroup(
					mWrist.setPIDCmd(WristConstants.Setpoints.L2SCORE)
							.andThen(mWrist.enableFFCmd()),
					new WaitCommand(0.1).andThen(mClaw.setClawCmd(-1.0)));
		} else if (curLevel == 0) {
			return new ParallelCommandGroup(
					mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
					mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
					mClaw.setClawCmd(ClawConstants.RollerSpeed.EJECT_ALGAE.get()));
		} else if (curLevel == 5) {
			return new SequentialCommandGroup(
					mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3),
					new ParallelCommandGroup(
							new SequentialCommandGroup(
									mElevator.setPIDCmd(
											ElevatorConstants.Setpoints.BARGE),
									mWrist.setPIDCmd(
											WristConstants.Setpoints.THROW_ALGAE)),
							mClaw.throwAlgae(mWrist, mElevator)));
		} else {

			return new ParallelCommandGroup(
					mWrist.setPIDCmd(WristConstants.Setpoints.SCORE).andThen(mWrist.enableFFCmd()),
					new WaitCommand(0.1).andThen(mClaw.setClawCmd(-1.0)));
		}
	}
}
