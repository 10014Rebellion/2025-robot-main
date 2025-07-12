package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.controls.StateTracker.AlgaeScoringLevel;
import frc.robot.subsystems.controls.StateTracker.CoralLevel;
import frc.robot.subsystems.controls.StateTracker.GamePiece;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser.SIDE;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.commands.DynamicCommand;

public class TeleopCommands {
	private final Drive mDrive;
	private final WristSubsystem mWrist;
	private final ElevatorSubsystem mElevator;
	private final IntakeSubsystem mIntake;
	private final ClawSubsystem mClaw;
	private final ClimbSubsystem mClimb;
	private final StateTracker mStateTracker;

	public TeleopCommands(
		Drive pDrive,
		WristSubsystem pWrist,
		ElevatorSubsystem pElevator,
		IntakeSubsystem pIntake,
		ClawSubsystem pClaw,
		ClimbSubsystem pClimb) {
		this.mDrive = pDrive;
		this.mWrist = pWrist;
		this.mElevator = pElevator;
		this.mIntake = pIntake;
		this.mClaw = pClaw;
		this.mClimb = pClimb;
		this.mStateTracker = new StateTracker();
	}

	public class ActionCommands {

		/** INTAKE CORAL: This command uses the intake, elevator, and wrist to intake the coral from the intake */
		public SequentialCommandGroup getIntakeCoralCmd() {
			return new SequentialCommandGroup(
				new ParallelDeadlineGroup( // End condition: when coral is in the cradle
					new ParallelCommandGroup(
						mIntake.setIndexCoralCmd(), // Turns on Indexer and ends when coral is in the cradle
						new SequentialCommandGroup(
							mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE), // Gets elevator into position
							mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE) // Brings wrist down
						)
					), 
					mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.INTAKING), // Deploys intake
					mIntake.setRollerCmd(IntakeConstants.IntakeRoller.kIntakeSpeed) // Turns on intake rollers
				),
				// AFTER CORAL IS DETECTED TO BE IN THE CRADLE \\
				new WaitCommand(0.1),
				new ParallelCommandGroup(
					mElevator.setPIDCmd(ElevatorConstants.Setpoints.POSTINTAKE), // Elevator lowers for wrist to pickup coral 
					mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE), // Wrist holds position to intake coral
					mClaw.intakeCoralCmd() // Claw intakes coral
				),
				mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE) // Elevator hightens after picking up coral
			);
		}

		/** OUTTAKE CORAL: This command is for EMERGENCY or if coral gets stuck*/
		public ParallelCommandGroup getOuttakeCoralCmd() {
			return new ParallelCommandGroup(
				mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.ALGAEINTAKE),
				mIntake.setIndexerCmd(-2),
				mIntake.setRollerCmd(-8)
			);
		}

		/** STOP INDEXER AND ROLLERS: Do you have a hard time reading camel case or something? This literally just stops the indexer and the intake rollers... just read the variable name- */
		public ParallelCommandGroup getStopIndexerAndRollersCmd(){
			return new ParallelCommandGroup(
				mIntake.setIndexerCmd(0),
				mIntake.setRollerCmd(0)
			);
		}

		/** STOW INTAKE: This command stows the intake and disables the rollers and indexer */
		public ParallelCommandGroup getStowIntakeCmd(){
			return new ParallelCommandGroup(
				mIntake
					.setEndablePIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.STOWED) // Sets intake pivot PID to stowed positon
					.andThen(mIntake.enableFFCmd()), // After PID is over, enables feedforward
					getStopIndexerAndRollersCmd()
			);
		}

		/** ALLIANCE RESET GYRO: Resets the rotation and flips rotation based on alliance color */
		public Command getAllianceResetGyroCmd(){
			return Commands.runOnce(
				() -> mDrive.resetGyro()
			).ignoringDisable(true);
		}
		
		/** TOGGLE IR: This command is for EMERGENCY, turns on/off the beambreaks,  */ 
		public InstantCommand getToggleIRCmd(){
			return new InstantCommand(() -> mIntake.toggleIRSensor());
		}

		/** GROUND ALGAE: Picks up the ground algae */
		public ParallelCommandGroup getGroundAlgaeCmd() {
			return new ParallelCommandGroup(
				mWrist
					.setPIDCmd(WristConstants.Setpoints.GROUNDALGAE)
					.andThen(mWrist.enableFFCmd()),
				new ParallelCommandGroup(
					mClaw.intakeCoralCmd(),
					mElevator.setPIDCmd(
						ElevatorConstants.Setpoints.GROUNDALGAE))
			);
		}

		/** HOLD ALGAE: Holds the algae with the arm */
		public ParallelCommandGroup getHoldAlgaeCmd() {
			return new ParallelCommandGroup(
			    new InstantCommand(() -> mStateTracker.setCurrentGamePiece(GamePiece.Algae)),
			    mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
			    mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE)
			        .andThen(mWrist.enableFFCmd()),
			    mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get())
			);
		}

		/** GO TO REEF: Goes to the reef to a given side, LEFT, RIGHT, or ALGAE (center) */
		public Command getGoToReefCmd(SIDE side) {
			return new SequentialCommandGroup(
				GoalPoseChooser.setSideCommand(side),
				mDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_CORAL)
			);
		}
		public Command getGoToAlgaeCmd(SIDE side) {
			return new SequentialCommandGroup(
				GoalPoseChooser.setSideCommand(side),
				mDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_ALGAE)
			);
		}

		/** STOP DRIVE: Stops the drive entirely, runs instantly */
		public Command getStopDriveCmd() {
			return mDrive.setDriveStateCommand(DriveState.TELEOP);
		}

		/////////////////// OPERATOR \\\\\\\\\\\\\\\\\\\\\
		/** SCORE CORAL: Scores the coral based on the current level */
		public DynamicCommand getScoreCoralCmd(){
			return new DynamicCommand(() -> getScoreCmd(mStateTracker.getCurrentCoralLevel()));
		}


		/** SCORE BARGE: Moves the wrist and chucks in the barge */
		public Command getScoreBargeCmd() {
			return new ParallelCommandGroup(
                new InstantCommand(() -> mStateTracker.setCurrentGamePiece(GamePiece.Algae)),
                new DynamicCommand(() -> getScoreCmd(AlgaeScoringLevel.NET)));
		}

		/** PREP CORAL SCORE CMD: */
		public Command getPrepCoralScoreCmd(CoralLevel pCoralLevel) {
			return new SequentialCommandGroup(
				new InstantCommand(() -> mStateTracker.setCurrentGamePiece(GamePiece.Coral)),
				new InstantCommand(() -> mStateTracker.setCurrentCoralLevel(pCoralLevel)),
				new DynamicCommand(() -> {
					if (pCoralLevel == CoralLevel.B1) {
						return new SequentialCommandGroup(
							mWrist.coralLevelToPIDCmd(pCoralLevel),
							mElevator.coralLevelToPIDCmd(pCoralLevel)
							
						);
					}
					return new ParallelCommandGroup(
						mElevator.coralLevelToPIDCmd(pCoralLevel),
						mWrist.coralLevelToPIDCmd(pCoralLevel)
					);
				})
			);
		}

		/** DO NOT USE THESE COMMANDS IN REAL MATCHES */
		public class ControllerTuningCommands {
			/** RESET GYRO: Basic rotation reset without accounting for alliance color */
			public final Command getResetGyroCmd(){
				return Commands.runOnce(() -> mDrive.resetGyro());
			}
		}
	}

	private Command getScoreCmd(CoralLevel pCoralLevel) {
		// Trough
		if (pCoralLevel == CoralLevel.T) {
			return mClaw.setClawCmd(ClawConstants.RollerSpeed.OUTTAKE_L1.get());
		
		// Branch 1 / Level 2
		} else if (pCoralLevel == CoralLevel.B1) {
			return new ParallelCommandGroup(
				mWrist.setPIDCmd(WristConstants.Setpoints.L2SCORE).andThen(mWrist.enableFFCmd()),
				new WaitCommand(0.1).andThen(mClaw.setClawCmd(-1.0))
			);
		
		// Branch 2 or 3 / Level 3 or 4
		} else {
			return new ParallelCommandGroup(
				mWrist.setPIDCmd(WristConstants.Setpoints.SCORE).andThen(mWrist.enableFFCmd()),
				new WaitCommand(0.1).andThen(mClaw.setClawCmd(-1.0)));
		}
  	}

	private Command getScoreCmd(AlgaeScoringLevel pAlgaeLevel) {
		// Net
	 	if (pAlgaeLevel == AlgaeScoringLevel.NET) {
			return new SequentialCommandGroup(
				mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3),
				// new ParallelCommandGroup(
				// 	// new WaitCommand(0.25).andThen(mClaw.setClawCmd(0.0)),
				// 	new SequentialCommandGroup(
				// 		mElevator.setPIDCmd(ElevatorConstants.Setpoints.BARGE),
				// 		mWrist.setPIDCmd(WristConstants.Setpoints.THROW_ALGAE)),
				// 	mClaw.throwAlgae(mWrist, mElevator)));
				new ParallelCommandGroup(
					// new WaitCommand(0.25).andThen(mClaw.setClawCmd(0.0)),
					mElevator.setPIDCmd(ElevatorConstants.Setpoints.BARGE),
					new SequentialCommandGroup(
						new WaitCommand(0.25),
						mWrist.setPIDCmd(WristConstants.Setpoints.THROW_ALGAE)),
					mClaw.throwAlgae(mWrist, mElevator)));
		}

		// Processor
		else {
			return new ParallelCommandGroup(
				mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
				mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
				mClaw.setClawCmd(ClawConstants.RollerSpeed.EJECT_ALGAE.get()));
		} 
	}
}
