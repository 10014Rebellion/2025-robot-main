package frc.robot.subsystems.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutonGoToPose;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawConstants.RollerSpeed;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionConstants.PoseOffsets;
import frc.robot.subsystems.vision.VisionConstants.linearPoseOffsets;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonSubsystem {
  private final DriveSubsystem mDrive;
  private final VisionSubsystem mVision;
  private final ClawSubsystem mClaw;
  private final ElevatorSubsystem mElevator;
  private final IntakeSubsystem mIntake;
  private final WristSubsystem mWrist;
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutonSubsystem(
      DriveSubsystem pDrive,
      WristSubsystem pWrist,
      VisionSubsystem pVision,
      ClawSubsystem pClaw,
      ElevatorSubsystem pElevator,
      IntakeSubsystem pIntake) {
    this.mDrive = pDrive;
    this.mWrist = pWrist;
    this.mVision = pVision;
    this.mClaw = pClaw;
    this.mElevator = pElevator;
    this.mIntake = pIntake;
    configureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    SmartDashboard.putData(autoChooser.getSendableChooser());

    // configureAutoChooser();
  }

  public Command getChosenAuton() {
    return autoChooser.get();
  }

  // private void configureAutoChooser() {
  // addSysIDRoutines();
  // }

  // private Command goToBranch() {
  //   return GoToPose(0, 0)
  // }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("ReadyScoreSubsystemsL1", readyScoreSubsystems(1));
    NamedCommands.registerCommand("ReadyScoreSubsystemsL2", readyScoreSubsystems(2));
    NamedCommands.registerCommand("ReadyScoreSubsystemsL3", readyScoreSubsystems(3));
    NamedCommands.registerCommand("ReadyScoreSubsystemsL4", readyScoreSubsystems(4));

    NamedCommands.registerCommand("ScoreCoral", scoreCoral());

    NamedCommands.registerCommand("IntakeCoral", intakeCoral());
    NamedCommands.registerCommand("ReadyHP", readyFunnelSubsystem());

    NamedCommands.registerCommand("ReadyAlgaeL3", readyAlgaeL3());
    NamedCommands.registerCommand("ReadyAlgaeL2", readyAlgaeL2());
    NamedCommands.registerCommand("ScoreBarge", scoreBarge());

    NamedCommands.registerCommand("HoldAlgae", holdAlgae());
    NamedCommands.registerCommand("HoldCoral", holdCoral());

    NamedCommands.registerCommand("DeployIntake", deployIntake());
    NamedCommands.registerCommand("ActivateIntake", activateIntake());

    NamedCommands.registerCommand("ScoreProcessor", scoreProcessor());
    NamedCommands.registerCommand("IntakeHP", HPCoralIntake());

    NamedCommands.registerCommand("GoToLeftPose", goToClosestBranchPose(false, 4));
    NamedCommands.registerCommand("GoToRightPose", goToClosestBranchPose(true, 4));
    NamedCommands.registerCommand("GoToCenterPose", goToClosestCenterPose());
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
                mIntake.setIndexCoralCmd(),
                // new WaitCommand(0.1),
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

  private ParallelCommandGroup holdAlgae() {
    return new ParallelCommandGroup(
      mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
      mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE).andThen(mWrist.enableFFCmd()),
      mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get()));
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
          mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3),
          new ParallelCommandGroup(
              // new WaitCommand(0.25).andThen(mClaw.setClawCmd(0.0)),
              mElevator.setPIDCmd(ElevatorConstants.Setpoints.BARGE),
              mWrist.setPIDCmd(WristConstants.Setpoints.THROW_ALGAE),
              mClaw.throwAlgae(mWrist, mElevator)));
  }

  private GoToPose goToPose(Pose2d targetPose) {
    return new GoToPose(() -> targetPose, () -> mDrive.getPose(), mDrive);
  }

  private GoToPose goToBranch(Pose2d targetPose) {
    return new GoToPose(() -> targetPose, () -> mDrive.getPose(), mDrive);
  }

  private AutonGoToPose goToClosestBranchPose(boolean isLeft, int level) {
    Supplier<linearPoseOffsets> horizontalOffset = () -> intToOffsets(level);
    PoseOffsets awayOffset = isLeft ? PoseOffsets.RIGHT : PoseOffsets.LEFT;

    return new AutonGoToPose(
        () -> mVision.getClosestReefScoringPose(horizontalOffset, () -> awayOffset),
        () -> mDrive.getPose(),
        mDrive);
  }

  private AutonGoToPose goToClosestCenterPose() {
    linearPoseOffsets awayOffset = intToOffsets(5);
    PoseOffsets sideOffset = PoseOffsets.CENTER;

    return new AutonGoToPose(
        () -> mVision.getClosestReefScoringPose(() -> awayOffset, () -> sideOffset),
        () -> mDrive.getPose(),
        mDrive);
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
  private SequentialCommandGroup HPCoralIntake() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            mIntake.setIndexCoralCmd(),
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
    if (level == 4) {
      return new ParallelCommandGroup(
          mElevator.setPIDCmd(ElevatorConstants.Setpoints.L4),
          mWrist.setPIDCmd(WristConstants.Setpoints.L4));
    }
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
                mIntake.setIndexCoralCmd(),
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
        mIntake.setIndexCoralCmd(),
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

  private linearPoseOffsets intToOffsets(int level) {
    int curLevel = MathUtil.clamp(level, 1, 5);
    switch (curLevel) {
      case 1:
        return linearPoseOffsets.L1;
      case 2:
        return linearPoseOffsets.L2;
      case 3:
        return linearPoseOffsets.L3;
      case 5:
        return linearPoseOffsets.ALGAE;
      default:
        return linearPoseOffsets.L4;
    }
  }

  public Command followFirstChoreoPath(String pathName, boolean correctStart, boolean correctEnd) {
    return followChoreoPath(pathName, correctStart, correctEnd, true);
  }

  public Command followChoreoPath(String pathName, boolean correctStart, boolean correctEnd) {
    return followChoreoPath(pathName, correctStart, correctEnd, false);
  }

  public Command followChoreoPath(
      String pathName, boolean correctStart, boolean correctEnd, boolean setPoseAtStart) {
    PathPlannerPath path =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? getChoreoTrajectory(pathName).get().flipPath()
            : getChoreoTrajectory(pathName).get();

    double totalTimeSeconds =
        path.getIdealTrajectory(DriveSubsystem.robotConfig).get().getTotalTimeSeconds();
    List<Pose2d> pathPoses = path.getPathPoses();
    Pose2d firstPose = pathPoses.get(0);
    Logger.recordOutput("Auton/Starting", firstPose);
    Pose2d lastPose = pathPoses.get(pathPoses.size() - 1);
    Logger.recordOutput("Auton/Ending", lastPose);

    Rotation2d startingRot = mDrive.getRotation();

    if (pathPoses.isEmpty()) {
      DriverStation.reportError(
          "CHOREO PATH ERROR: PATH " + pathName + " DOESN'T HAVE ANY POSES", true);
      return AutoBuilder.followPath(path).withTimeout(totalTimeSeconds + 0.5);
    }

    return new SequentialCommandGroup(
        (setPoseAtStart)
            ? new InstantCommand(
                () -> {
                  mDrive.setPose(
                      AllianceFlipUtil.apply(new Pose2d(firstPose.getTranslation(), startingRot)));
                })
            : new InstantCommand(),
        (correctStart)
            ? goToPose(
                AllianceFlipUtil.apply(
                    new Pose2d(firstPose.getTranslation(), firstPose.getRotation())))
            : new InstantCommand(),
        AutoBuilder.followPath(path).withTimeout(totalTimeSeconds + 0.5),
        (correctEnd)
            ? goToPose(
                AllianceFlipUtil.apply(
                    new Pose2d(lastPose.getTranslation(), lastPose.getRotation())))
            : new InstantCommand());
  }

  public Optional<PathPlannerPath> getChoreoTrajectory(String pathName) {
    try {
      return Optional.of(PathPlannerPath.fromChoreoTrajectory(pathName));
    } catch (Exception e) {
      e.printStackTrace();
      return Optional.empty();
    }
  }

  private void addSysIDRoutines() {
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(mDrive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(mDrive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        mDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        mDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", mDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", mDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
