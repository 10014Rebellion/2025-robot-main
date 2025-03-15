package frc.robot.subsystems.auton;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.claw.ClawConstants.RollerSpeed;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.vision.VisionConstants.PoseOffsets;
import frc.robot.subsystems.vision.VisionConstants.linearPoseOffsets;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.function.Supplier;

public class AutonSubsystem {
  private final DriveSubsystem mDrive;
  private final VisionSubsystem mVision;
  private final ClawSubsystem mClaw;
  private final ElevatorSubsystem mElevator;
  private final PivotSubsystem mPivot;
  private final IntakeSubsystem mIntake;
  private final WristSubsystem mWrist;

  public AutonSubsystem(
      DriveSubsystem pDrive,
      WristSubsystem pWrist,
      VisionSubsystem pVision,
      ClawSubsystem pClaw,
      ElevatorSubsystem pElevator,
      PivotSubsystem pPivot,
      IntakeSubsystem pIntake) {
    this.mDrive = pDrive;
    this.mWrist = pWrist;
    this.mVision = pVision;
    this.mClaw = pClaw;
    this.mElevator = pElevator;
    this.mPivot = pPivot;
    this.mIntake = pIntake;
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("ReadyScoreSubsystemsL1", readyScoreSubsystems(1));
    NamedCommands.registerCommand("ReadyScoreSubsystemsL2", readyScoreSubsystems(2));
    NamedCommands.registerCommand("ReadyScoreSubsystemsL3", readyScoreSubsystems(3));
    NamedCommands.registerCommand("ReadyScoreSubsystemsL4", readyScoreSubsystems(4));

    NamedCommands.registerCommand("ScoreCoral", scoreCoral());

    NamedCommands.registerCommand("IntakeCoral", intakeCoral());

    NamedCommands.registerCommand("ScoreToPoseC10L2", GoToPose(10, 2));
    NamedCommands.registerCommand("ScoreToPoseC3L2", GoToPose(3, 2));
    NamedCommands.registerCommand("ScoreToPoseC10L4", GoToPose(10, 4));
    NamedCommands.registerCommand("ScoreToPoseC3L4", GoToPose(3, 4));

    NamedCommands.registerCommand("ReadyAlgaeL3", readyAlgaeL3());
    NamedCommands.registerCommand("ReadyAlgaeL2", readyAlgaeL2());

    NamedCommands.registerCommand("HoldAlgae", holdAlgae());

    NamedCommands.registerCommand("ScoreProcessor", scoreProcessor());
  }

  private SequentialCommandGroup scoreProcessor() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> mClaw.setClaw(RollerSpeed.OUTTAKE_PROCESSOR)),
        new WaitCommand(3),
        new InstantCommand(() -> mClaw.setClaw(0)));
  }

  private ParallelCommandGroup holdAlgae() {
    return new ParallelCommandGroup(
        mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
        mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
        new InstantCommand(() -> mClaw.setClaw(RollerSpeed.HOLD_ALGAE)));
  }

  private ParallelCommandGroup readyAlgaeL3() {
    return new ParallelCommandGroup(
        mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3ALGAE),
        mWrist.setPIDCmd(WristConstants.Setpoints.L3ALGAE),
        new InstantCommand(() -> mClaw.setClaw(RollerSpeed.INTAKE_ALGAE)));
  }

  private ParallelCommandGroup readyAlgaeL2() {
    return new ParallelCommandGroup(
        mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2ALGAE),
        mWrist.setPIDCmd(WristConstants.Setpoints.L2ALGAE),
        new InstantCommand(() -> mClaw.setClaw(RollerSpeed.INTAKE_ALGAE)));
  }

  private SequentialCommandGroup GoToPose(int branch, int level) {
    Supplier<linearPoseOffsets> horizontalOffset = () -> intToOffsets(level);
    Supplier<PoseOffsets> awayOffset =
        () -> ((branch % 2 == 0) ? PoseOffsets.RIGHT : PoseOffsets.LEFT);

    return new SequentialCommandGroup(
        new InstantCommand(() -> mDrive.stop()),
        new GoToPose(
            () -> mVision.getClosestReefScoringPose(horizontalOffset, awayOffset),
            () -> mDrive.getPose(),
            mDrive),
        new WaitCommand(0.5),
        scoreCoral());
  }

  // private SequentialCommandGroup GoToPose(String pathToFollowAfter, int tagID,
  // double distAway, double distHorizontal) {
  // try {
  // PathPlannerPath pathToGoal = PathPlannerPath.fromPathFile(pathToFollowAfter);

  // return new SequentialCommandGroup(
  // new InstantCommand(() -> mDrive.stopAllCommands()),
  // new GoToPose(
  // () -> mVision.getPoseInFrontOfAprilTag(tagID, distAway, distHorizontal),
  // () -> mDrive.getPose(),
  // mDrive),
  // AutoBuilder.followPath(pathToGoal));
  // } catch (Exception e) {
  // return new SequentialCommandGroup(
  // new InstantCommand(() -> System.out.println("<<FAILED TO FIND GoToPose
  // AUTONS>>")));
  // }
  // }

  private SequentialCommandGroup readyScoreSubsystems(int level) {
    return new SequentialCommandGroup(
        mWrist.setPIDCmd(intToWristPos(level)), mElevator.setPIDCmd(intToElevatorPos(level)));
  }

  private SequentialCommandGroup scoreCoral() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            mWrist.setPIDCmd(WristConstants.Setpoints.SCORE),
            mElevator.setPIDCmd(ElevatorConstants.Setpoints.SCORE)),
        mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE));
  }

  private SequentialCommandGroup intakeCoral() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                mElevator.setPIDCmd((ElevatorConstants.Setpoints.PREINTAKE)),
                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE))),
        new SequentialCommandGroup(
            mElevator.setPIDCmd((ElevatorConstants.Setpoints.PREINTAKE)),
            mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE)),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                mElevator.setPIDCmd((ElevatorConstants.Setpoints.POSTINTAKE)),
                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE))));
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

  private linearPoseOffsets intToOffsets(int level) {
    int curLevel = MathUtil.clamp(level, 1, 4);
    switch (curLevel) {
      case 1:
        return linearPoseOffsets.L1;
      case 2:
        return linearPoseOffsets.L2;
      case 3:
        return linearPoseOffsets.L3;
      default:
        return linearPoseOffsets.L4;
    }
  }
}
