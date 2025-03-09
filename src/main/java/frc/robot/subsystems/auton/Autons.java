package frc.robot.subsystems.auton;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;
import frc.robot.subsystems.claw.ClawFFCommand;
import frc.robot.subsystems.claw.ClawIntakeCoralCommand;
import frc.robot.subsystems.claw.ClawPIDCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorFFCommand;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.elevatorPivot.ElevatorPivot;
import frc.robot.subsystems.intake.OTBIntake;
import frc.robot.subsystems.intake.autoIntakeCoralCommand;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.PoseOffsets;
import frc.robot.subsystems.vision.VisionConstants.linearPoseOffsets;
import java.util.function.Supplier;

public class Autons {
  private final Drive mDrive;
  private final Vision mVision;
  private final Claw mClaw;
  private final Elevator mElevator;
  private final ElevatorPivot mPivot;
  private final OTBIntake mIntake;

  public Autons(
      Drive pDrive,
      Vision pVision,
      Claw pClaw,
      Elevator pElevator,
      ElevatorPivot pPivot,
      OTBIntake pIntake) {
    this.mDrive = pDrive;
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

    NamedCommands.registerCommand("HoldElevatorAndWrist", activateElevatorWristFF());

    NamedCommands.registerCommand("IntakeCoral", intakeCoral());

    NamedCommands.registerCommand("ScoreToPoseC10L2", GoToPose(10, 2));
    NamedCommands.registerCommand("ScoreToPoseC3L2", GoToPose(3, 2));
    NamedCommands.registerCommand("ScoreToPoseC10L4", GoToPose(10, 4));
    NamedCommands.registerCommand("ScoreToPoseC3L4", GoToPose(3, 4));

    NamedCommands.registerCommand("ReadyAlgaeL3", readyAlgaeL3());

    NamedCommands.registerCommand("HoldAlgae", holdAlgae());

    NamedCommands.registerCommand("ScoreProcessor", scoreProcessor());
  }

  private SequentialCommandGroup scoreProcessor() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> mClaw.setClaw(ClawConstants.Claw.ClawRollerVolt.OUTTAKE_PROCESSOR)),
        new WaitCommand(3),
        new InstantCommand(() -> mClaw.setClaw(0)));
  }

  private ParallelCommandGroup holdAlgae() {
    return new ParallelCommandGroup(
        new ElevatorPIDCommand(ElevatorConstants.Positions.HOLD_ALGAE, mElevator),
        new ClawPIDCommand(ClawConstants.Wrist.Positions.HOLD_ALGAE, mClaw),
        new InstantCommand(() -> mClaw.setClaw(ClawRollerVolt.HOLD_ALGAE)));
  }

  private ParallelCommandGroup readyAlgaeL3() {
    return new ParallelCommandGroup(
        new ElevatorPIDCommand(ElevatorConstants.Positions.L3ALGAE, mElevator),
        new ClawPIDCommand(ClawConstants.Wrist.Positions.L3ALGAE, mClaw),
        new InstantCommand(() -> mClaw.setClaw(ClawRollerVolt.INTAKE_ALGAE)));
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

  private ParallelCommandGroup activateElevatorWristFF() {
    return new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator));
  }

  private SequentialCommandGroup readyScoreSubsystems(int level) {
    return new SequentialCommandGroup(
        new ClawPIDCommand(intToWristPos(level), mClaw),
        new ElevatorPIDCommand(intToElevatorPos(level), mElevator));
  }

  private SequentialCommandGroup scoreCoral() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ClawPIDCommand(ClawConstants.Wrist.Positions.SCORE, mClaw),
            new ElevatorPIDCommand(ElevatorConstants.Positions.SCORE, mElevator)),
        new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw));
  }

  private SequentialCommandGroup intakeCoral() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new autoIntakeCoralCommand(mIntake),
            new SequentialCommandGroup(
                new ElevatorPIDCommand((ElevatorConstants.Positions.PREINTAKE), mElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw))),
        new SequentialCommandGroup(
            new ElevatorPIDCommand((ElevatorConstants.Positions.PREINTAKE), mElevator),
            new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw)),
        new ParallelCommandGroup(
            new ClawIntakeCoralCommand(mClaw),
            new SequentialCommandGroup(
                new ElevatorPIDCommand((ElevatorConstants.Positions.POSTINTAKE), mElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw))));
  }

  private ClawConstants.Wrist.Positions intToWristPos(int level) {
    int curLevel = MathUtil.clamp(level, 1, 4);
    switch (curLevel) {
      case 1:
        return ClawConstants.Wrist.Positions.L1;
      case 2:
        return ClawConstants.Wrist.Positions.L2;
      case 3:
        return ClawConstants.Wrist.Positions.L3;
      default:
        return ClawConstants.Wrist.Positions.L4;
    }
  }

  private ElevatorConstants.Positions intToElevatorPos(int level) {
    int curLevel = MathUtil.clamp(level, 1, 4);
    switch (curLevel) {
      case 1:
        return ElevatorConstants.Positions.L1;
      case 2:
        return ElevatorConstants.Positions.L2;
      case 3:
        return ElevatorConstants.Positions.L3;
      default:
        return ElevatorConstants.Positions.L4;
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
