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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.controls.ControlsConstants.Buttonboard.Direction;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionConstants.PoseOffsets;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.arithmetic.AllianceFlipUtil;

public class TeleopCommands {
	private final DriveSubsystem mDrive;
	private final VisionSubsystem mVision;
	private final WristSubsystem mWrist;
	private final ElevatorSubsystem mElevator;
	private final IntakeSubsystem mIntake;
	private final ClawSubsystem mClaw;
	private final ClimbSubsystem mClimb;

	public TeleopCommands(
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
	}

	public class ControllerCommands {
		public Command getDriveCmd(CommandXboxController pDriverController, boolean isFieldOriented){
			return getDriveCmd(pDriverController, isFieldOriented, false);
		}

		public Command getDriveCmd(CommandXboxController pDriverController, boolean isFieldOriented, boolean isAligningToBarge){
			return DriveCommands.joystickDrive(
				mDrive,
				() ->  isAligningToBarge ? 0.0 : -pDriverController.getLeftY(),
				() -> -pDriverController.getLeftX(),
				() -> -pDriverController.getRightX(),
				() -> isFieldOriented
			);
		}

		public GoToPose getGoToPoseCmd(PoseOffsets poseOffset) {
			return new GoToPose(
				() -> mVision.getClosestReefScoringPose(poseOffset),
				() -> mDrive.getPose(),
			mDrive);
		}

		/** INTAKE CORAL: This command uses the intake, elevator, and wrist to intake the coral from the intake */
		public final SequentialCommandGroup intakeCoral = new SequentialCommandGroup(
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

		/** OUTTAKE CORAL: This command is for EMERGENCY or if coral gets stuck*/
		public final ParallelCommandGroup outtakeCoral = new ParallelCommandGroup(
			mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.ALGAEINTAKE),
			mIntake.setIndexerCmd(-2),
			mIntake.setRollerCmd(-8)
		);

		/** STOP INDEXER AND ROLLERS: Do you have a hard time reading camel case or something? This literally just stops the indexer and the intake rollers... just read the variable name- */
		public final ParallelCommandGroup stopIndexerAndRollers = new ParallelCommandGroup(
			mIntake.setIndexerCmd(0),
			mIntake.setRollerCmd(0)
		);

		/** STOW INTAKE: This command stows the intake and disables the rollers and indexer */
		public final ParallelCommandGroup stowIntake = new ParallelCommandGroup(
			mIntake
				.setEndablePIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.STOWED) // Sets intake pivot PID to stowed positon
				.andThen(mIntake.enableFFCmd()), // After PID is over, enables feedforward
			stopIndexerAndRollers
		); 

		/** ALLIANCE RESET GYRO: Resets the rotation and flips rotation based on alliance color */
		public final Command allianceResetGyro = Commands.runOnce(
			() -> mDrive.setPose(
				new Pose2d(
					mDrive.getPose().getTranslation(), // Keeps current translation
					AllianceFlipUtil.apply(new Rotation2d()) // Resets rotation and flips it if its on red alliance
				)
			),mDrive
		).ignoringDisable(true);

		
		/** DO NOT USE THESE COMMANDS IN REAL MATCHES */
		public class ControllerTuningCommands {
			public Command getDriveStraightCmd(Direction desiredDirection){
				double yMovement = 
					(desiredDirection == Direction.FORWARD) 
						? -1 
						: (desiredDirection == Direction.BACKWARD) 
							? 1 
							: 0;

				double xMovement = (desiredDirection == Direction.LEFT) ? -1 : 1;

				return DriveCommands.joystickDrive(mDrive, () -> yMovement, () -> 0, () -> xMovement, () -> false);
			} 

			/** RESET GYRO: Basic rotation reset without accounting for alliance color */
			public final Command resetGyro = Commands.runOnce(
				() -> mDrive.setPose(
					new Pose2d(
						mDrive.getPose().getTranslation(), // Keeps current translation
						new Rotation2d() // Resets rotation 
					)
				),mDrive
			).ignoringDisable(true);
		}
	}

	
}
