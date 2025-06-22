package frc.robot.subsystems.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
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
import frc.robot.util.math.AllianceFlipUtil;

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
				() -> mDrive.setPose(
					new Pose2d(
						mDrive.getPoseEstimate().getTranslation(), // Keeps current translation
						AllianceFlipUtil.apply(new Rotation2d()) // Resets rotation and flips it if its on red alliance
					)
				),mDrive
			).ignoringDisable(true);
		}
		
		/** TOGGLE IR: This command is for EMERGENCY, turns on/off the beambreaks,  */ 
		public InstantCommand getToggleIRCmd(){
			return new InstantCommand(() -> mIntake.toggleIRSensor());
		}

		/** */
		public ParallelCommandGroup getGroundAlgae() {
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

		/**  */
		public Command getGoToReefCmd(SIDE side) {
			return new SequentialCommandGroup(
				GoalPoseChooser.setSideCommand(side),
				mDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_CORAL)
			);
		}

		/** */
		public Command getStopDriveCmd() {
			return mDrive.setDriveStateCommand(DriveState.TELEOP);
		}

    //       new ParallelCommandGroup(
    //           new InstantCommand(() -> currentScoreLevel = 0),
    //           mElevator.setPIDCmd(
    //               ElevatorConstants.Setpoints.HOLD_ALGAE),
    //           mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE)
    //               .andThen(mWrist.enableFFCmd()),
    //           mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE
    //               .get())));

		/** DO NOT USE THESE COMMANDS IN REAL MATCHES */
		public class ControllerTuningCommands {
			/** RESET GYRO: Basic rotation reset without accounting for alliance color */
			public final Command getResetGyroCmd(){
				return Commands.runOnce(() -> mDrive.resetGyro());
			}
		}



	}	
}
