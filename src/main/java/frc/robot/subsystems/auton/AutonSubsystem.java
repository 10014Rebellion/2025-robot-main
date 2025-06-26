package frc.robot.subsystems.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawConstants.RollerSpeed;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonSubsystem {
  private final Drive mDrive;
  private final ClawSubsystem mClaw;
  private final ElevatorSubsystem mElevator;
  private final IntakeSubsystem mIntake;
  private final WristSubsystem mWrist;
  private final LoggedDashboardChooser<Command> autoChooser;

    public AutonSubsystem(
        Drive pDrive,
        WristSubsystem pWrist,
        ClawSubsystem pClaw,
        ElevatorSubsystem pElevator,
        IntakeSubsystem pIntake) {
        this.mDrive = pDrive;
        this.mWrist = pWrist;
        this.mClaw = pClaw;
        this.mElevator = pElevator;
        this.mIntake = pIntake;
        configureNamedCommands();

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        autoChooser.addDefaultOption("Taxi", mDrive.setDriveStateCommand(DriveState.UP));
        SmartDashboard.putData(autoChooser.getSendableChooser());
    }

    public Command getChosenAuton() {
        return autoChooser.get();
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("ReadyScoreSubsystemsL1", readyScoreSubsystems(1));
        NamedCommands.registerCommand("ReadyScoreSubsystemsL2", readyScoreSubsystems(2));
        NamedCommands.registerCommand("ReadyScoreSubsystemsL3", readyScoreSubsystems(3));
        NamedCommands.registerCommand("ReadyScoreSubsystemsL4", readyScoreSubsystems(4));

        NamedCommands.registerCommand("ScoreCoral", scoreCoral());
        NamedCommands.registerCommand("ScoreBarge", scoreBarge());

        NamedCommands.registerCommand("IntakeCoral", intakeCoral());
        NamedCommands.registerCommand("ReadyHP", readyFunnelSubsystem());

        NamedCommands.registerCommand("ReadyAlgaeL3", readyAlgaeL3());
        NamedCommands.registerCommand("ReadyAlgaeL2", readyAlgaeL2());

        NamedCommands.registerCommand("HoldAlgae", holdAlgae());
        NamedCommands.registerCommand("HoldCoral", holdCoral());

        NamedCommands.registerCommand("DeployIntake", deployIntake());
        NamedCommands.registerCommand("ActivateIntake", activateIntake());

        NamedCommands.registerCommand("ScoreProcessor", scoreProcessor());
        NamedCommands.registerCommand("IntakeHP", HPCoralIntake());

        NamedCommands.registerCommand("GoToLeftPose", goToClosestBranchPose(true, 4));
        NamedCommands.registerCommand("GoToRightPose", goToClosestBranchPose(false, 4));
        NamedCommands.registerCommand("GoToCenterPose", goToClosestCenterPose());

        NamedCommands.registerCommand("STOP", mDrive.setDriveStateCommand(DriveState.STOP));
    }

    private InstantCommand holdCoral() {
        return new InstantCommand(() -> mClaw.setClaw(ClawConstants.RollerSpeed.HOLD_CORAL));
    }

    private ParallelRaceGroup deployIntake() {
        return new ParallelRaceGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.3),
                new ParallelCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE),
                    mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE))),
            mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.INTAKING));
    }

    private SequentialCommandGroup activateIntake() {
        return new ParallelRaceGroup(
            mIntake.setRollerCmd(IntakeConstants.IntakeRoller.kIntakeSpeed),
            mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.INTAKING),
            new SequentialCommandGroup(
                mIntake.autonSetIndexCoralCmd(),
                new ParallelCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.POSTINTAKE),
                    mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE),
                    mClaw.intakeCoralCmd()),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE)))
            .andThen(new InstantCommand(() -> mIntake.setVoltsIntakeRoller(0)));
    }

    private SequentialCommandGroup scoreProcessor() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> mClaw.setClaw(RollerSpeed.OUTTAKE_PROCESSOR)),
            new WaitCommand(3),
            new InstantCommand(() -> mClaw.setClaw(0)));
    }

    private SequentialCommandGroup holdAlgae() {
        return new SequentialCommandGroup(
            new WaitCommand(0.3),
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE).andThen(mWrist.enableFFCmd()),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get())));
    }

    private ParallelCommandGroup readyAlgaeL3() {
        return new ParallelCommandGroup(
            mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3ALGAE),
            mWrist.setPIDCmd(WristConstants.Setpoints.L3ALGAE).andThen(mWrist.enableFFCmd()),
            mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get()));
    }

    private ParallelCommandGroup readyAlgaeL2() {
        return new ParallelCommandGroup(
            mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2ALGAE),
            mWrist.setPIDCmd(WristConstants.Setpoints.L2ALGAE).andThen(mWrist.enableFFCmd()),
            mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get()));
    }

    private SequentialCommandGroup scoreBarge() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get())),
            mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3),
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.BARGE),
                mWrist.setPIDCmd(WristConstants.Setpoints.THROW_ALGAE),
                mClaw.autonThrowAlgae(mWrist, mElevator)));
    }

    private Command goToClosestBranchPose(boolean isLeft, int level) {
        GoalPoseChooser.SIDE branchOffset = isLeft ? GoalPoseChooser.SIDE.LEFT : GoalPoseChooser.SIDE.RIGHT;
        GoalPoseChooser.setSide(branchOffset);

        return Commands.runOnce(() -> GoalPoseChooser.setSide( isLeft ? GoalPoseChooser.SIDE.LEFT : GoalPoseChooser.SIDE.RIGHT)).andThen(mDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_CORAL)
            .withDeadline(mDrive.waitUnitllIntakeAutoAlignFinishes()));
    }

    private Command goToClosestCenterPose() {
        GoalPoseChooser.SIDE branchOffset = GoalPoseChooser.SIDE.ALGAE;
        GoalPoseChooser.setSide(branchOffset);

        return mDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_CORAL)
            .withDeadline(mDrive.waitUnitllAutoAlignFinishes());
    }

    private SequentialCommandGroup HPCoralIntake() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                mIntake.autonSetIndexCoralCmd(),
                new SequentialCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE),
                    mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE))),
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.POSTINTAKE),
                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE),
                mClaw.intakeCoralCmd()),
            mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE));
    }

    private ParallelCommandGroup readyScoreSubsystems(int level) {
        return new ParallelCommandGroup(
            mWrist.setPIDCmd(intToWristPos(level)), mElevator.setPIDCmd(intToElevatorPos(level)));
    }

    private ParallelDeadlineGroup scoreCoral() {
        return new ParallelDeadlineGroup(
            new WaitCommand(0.5),
            mWrist.setPIDCmd(WristConstants.Setpoints.SCORE).andThen(mWrist.enableFFCmd()),
            new WaitCommand(0.1).andThen(mClaw.setClawCmd(0.0)));
    }

    private SequentialCommandGroup intakeCoral() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    mIntake.autonSetIndexCoralCmd(),
                    new SequentialCommandGroup(
                        mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE),
                        mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE))),
                mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.INTAKING),
                mIntake.setRollerCmd(8)),
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.POSTINTAKE),
                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE),
                mClaw.intakeCoralCmd()),
            mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE));
    }

    private ParallelCommandGroup readyFunnelSubsystem() {
        return new ParallelCommandGroup(
            mIntake.autonSetIndexCoralCmd(),
            mElevator.setPIDCmd(ElevatorConstants.Setpoints.HPINTAKE),
            mWrist.setPIDCmd(WristConstants.Setpoints.HPINTAKE));
    }

    private WristConstants.Setpoints intToWristPos(int level) {
        int curLevel = MathUtil.clamp(level, 1, 4);
        switch (curLevel) {
            case 1:
                return WristConstants.Setpoints.L1;
            case 2:
                return WristConstants.Setpoints.L2;
            case 3:
            return WristConstants.Setpoints.L3;
        default:
            return WristConstants.Setpoints.L4;
        }
    }

    private ElevatorConstants.Setpoints intToElevatorPos(int level) {
        int curLevel = MathUtil.clamp(level, 1, 4);
        switch (curLevel) {
            case 1:
                return ElevatorConstants.Setpoints.L1;
            case 2:
                return ElevatorConstants.Setpoints.L2;
            case 3:
                return ElevatorConstants.Setpoints.L3;
            default:
                return ElevatorConstants.Setpoints.L4;
        }
    }

//   private linearPoseOffsets intToOffsets(int level) {
//     int curLevel = MathUtil.clamp(level, 1, 5);
//     switch (curLevel) {
//       case 1:
//         return linearPoseOffsets.L1;
//       case 2:
//         return linearPoseOffsets.L2;
//       case 3:
//         return linearPoseOffsets.L3;
//       case 5:
//         return linearPoseOffsets.ALGAE;
//       default:
//         return linearPoseOffsets.L4;
//     }
//   }

//   private void addSysIDRoutines() {
//     // Set up SysId routines
//     autoChooser.addOption(
//         "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(mDrive));
//     autoChooser.addOption(
//         "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(mDrive));
//     autoChooser.addOption(
//         "Drive SysId (Quasistatic Forward)",
//         mDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//     autoChooser.addOption(
//         "Drive SysId (Quasistatic Reverse)",
//         mDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//     autoChooser.addOption(
//         "Drive SysId (Dynamic Forward)", mDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
//     autoChooser.addOption(
//         "Drive SysId (Dynamic Reverse)", mDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
//   }
}
