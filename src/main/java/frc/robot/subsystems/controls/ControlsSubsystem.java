// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.DynamicCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ControlsSubsystem extends SubsystemBase {
  private final CommandXboxController driverController = new CommandXboxController(0);
  // private final CommandXboxController operatorController = new
  // CommandXboxController(1);
  private final CommandGenericHID operatorButtonboard = new CommandGenericHID(1);

  private int currentScoreLevel;
  private Supplier<VisionConstants.PoseOffsets> sideScoring;
  private boolean mSwerveFieldOriented = true;
  private boolean doAutoScore = true;
  private boolean goingToBarge = false;

  // private final double kDriveSwerveMultipler = 0.5;
  // private final double kRotationSwerveMultipler = 0.6;

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
    SmartDashboard.putBoolean("IS FIELD ORIENTED", mSwerveFieldOriented);
  }

  public void initTriggers() {
    new Trigger(
            () ->
                (mDrive.isAtPose && mElevator.isPIDAtGoal() && mWrist.isPIDAtGoal() && doAutoScore))
        .whileTrue(new DynamicCommand(() -> getScoreCmd(currentScoreLevel)));

    // new Trigger(
    // () ->
    // (goingToBarge
    // && mElevator.getEncReading() > ElevatorConstants.throwAlgaePos
    // && (mWrist.getEncReading() > WristConstants.throwAlgaePos)))
    // .whileTrue(
    // new InstantCommand(() ->
    // mClaw.setClaw(ClawConstants.RollerSpeed.EJECT_ALGAE.get())));

    // new Trigger(
    // () ->
    // ( // (currentScoreLevel == 5) // Check we're scoring barge
    // (goingToBarge)
    // && // ALso check that we should actually have an algae
    // (mElevator.getEncReading()
    // > ElevatorConstants
    // .throwAlgaePos) // Now, check if we're high enough on the elevator
    // && (mWrist.getEncReading()
    // > WristConstants
    // .throwAlgaePos) // Also check if the arm is high enough to throw
    // ))
    // .whileTrue(
    // (new InstantCommand(() -> goingToBarge = false))
    // .andThen(mClaw.setClawCmd(ClawConstants.RollerSpeed.EJECT_ALGAE.get())))
    // .whileFalse(mClaw.setClawCmd(0.0));
    // .alongWith(new InstantCommand(() -> hasAlgae = mClaw.getBeamBreak())));
    // .whileFalse(new InstantCommand(() -> setSolid(defaultColor)));
  }

  public void initDriverController() {
    // driverController
    // .povUp()
    // .whileTrue(
    // new InstantCommand(
    // () ->
    // mClimb.setGrabberVolts(
    // ClimbConstants.Grabber.VoltageSetpoints.PULL_IN.getVolts())));
    driverController
        .rightBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new ParallelCommandGroup(
                        mIntake.setIndexCoralCmd(),
                        new SequentialCommandGroup(
                            mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE),
                            mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE))),
                    mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.INTAKING),
                    mIntake.setRollerCmd(IntakeConstants.IntakeRoller.kIntakeSpeed)),
                new WaitCommand(0.1),
                new ParallelCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.POSTINTAKE),
                    mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE),
                    mClaw.intakeCoralCmd()),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE)))
        .whileFalse(
            new ParallelCommandGroup(
                mIntake
                    .setEndablePIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.STOWED)
                    .andThen(mIntake.enableFFCmd()),
                mIntake.setIndexerCmd(0),
                mIntake.setRollerCmd(0)));
    // mElevator.enableFFCmd(),
    // mWrist.enableFFCmd()));
    driverController
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.ALGAEINTAKE),
                mIntake.setIndexerCmd(-2),
                mIntake.setRollerCmd(-8)))
        .whileFalse(new ParallelCommandGroup(mIntake.setIndexerCmd(0), mIntake.setRollerCmd(0)));
    driverController
        .y()
        .whileTrue(
            new ParallelCommandGroup(
                mWrist
                    .setPIDCmd(WristConstants.Setpoints.GROUNDALGAE)
                    .andThen(mWrist.enableFFCmd()),
                new ParallelCommandGroup(
                    mClaw.intakeCoralCmd(),
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.GROUNDALGAE))))
        .whileFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> currentScoreLevel = 0),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE).andThen(mWrist.enableFFCmd()),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get())));

    driverController.povRight().onTrue(new InstantCommand(() -> doAutoScore = !doAutoScore));

    driverController
        .povUp()
        .whileTrue(mClimb.setGrabberVoltsCmd(ClimbConstants.Grabber.VoltageSetpoints.PULL_IN));
    driverController.povDown().whileTrue(mClimb.retractClimb());
  }

  public void initDrivebase() {
    mDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            mDrive,
            () -> isAligningToBarge ? 0.0 : -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> mSwerveFieldOriented));

    driverController
        .b()
        .onTrue(new InstantCommand(() -> mSwerveFieldOriented = !mSwerveFieldOriented));

    driverController
        .rightTrigger()
        .whileTrue(
            new GoToPose(
                () -> mVision.getClosestReefScoringPose(VisionConstants.PoseOffsets.RIGHT),
                () -> mDrive.getPose(),
                mDrive));

    driverController
        .leftTrigger()
        .whileTrue(
            new GoToPose(
                () -> mVision.getClosestReefScoringPose(VisionConstants.PoseOffsets.LEFT),
                () -> mDrive.getPose(),
                mDrive));

    driverController
        .a()
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> doAutoScore = false),
                new GoToPose(
                    () -> mVision.getClosestReefScoringPose(VisionConstants.PoseOffsets.CENTER),
                    () -> mDrive.getPose(),
                    mDrive)))
        .whileFalse(new InstantCommand(() -> doAutoScore = true));

    // driverController
    //     .x()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     mDrive.setPose(
    //                         new Pose2d(mDrive.getPose().getTranslation(), new Rotation2d())),
    //                 mDrive)
    //             .ignoringDisable(true));

    driverController
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        mDrive.setPose(
                            new Pose2d(
                                mDrive.getPose().getTranslation(),
                                new Rotation2d()
                                    .plus(
                                        ((DriverStation.getAlliance().orElse(Alliance.Blue)
                                                == Alliance.Red)
                                            ? Rotation2d.kPi
                                            : Rotation2d.kZero)))),
                    mDrive)
                .ignoringDisable(true));

    // driverController
    //     .x()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(() -> isAligningToBarge = true),
    //             new GoToBarge(
    //                 () -> {
    //                   double currentY = mDrive.getPose().getY();
    //                   double currentX = mDrive.getPose().getX();

    //                   double blueDiff =
    //                       Math.abs(currentX - PoseConstants.WeldedField.kBargeAlignBlueX);
    //                   double redDiff =
    //                       Math.abs(currentX - PoseConstants.WeldedField.kBargeAlignRedX);

    //                   double targetX;
    //                   double targetThetaDeg;

    //                   if (redDiff < blueDiff) {
    //                     targetX = PoseConstants.WeldedField.kBargeAlignRedX;
    //                     targetThetaDeg = PoseConstants.WeldedField.kBargeAlignEastDeg;
    //                   } else {
    //                     targetX = PoseConstants.WeldedField.kBargeAlignBlueX;
    //                     targetThetaDeg = PoseConstants.WeldedField.kBargeAlignWestDeg;
    //                   }

    //                   targetThetaDeg *= (Robot.gIsBlueAlliance) ? -1 : 1;

    //                   return new Pose2d(
    //                       targetX,
    //                       currentY,
    //                       new Rotation2d(Units.degreesToRadians(targetThetaDeg)));
    //                 },
    //                 () -> mDrive.getPose(),
    //                 mDrive)))
    //     .onFalse(new InstantCommand(() -> isAligningToBarge = false));

    // Menu button, the one with 3 lines like a hamburger menu icon
    driverController.button(8).onTrue(new InstantCommand(() -> mIntake.toggleIRSensor()));

    // Weird button with the two rectangles inside each other, idk what its called ngl
    // driverController.button(7).onTrue(new InstantCommand(() -> mIntake.toggleIRSensor()));
  }

  public void initDriveTuning() {
    mDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            mDrive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> mSwerveFieldOriented));

    driverController
        .b()
        .onTrue(new InstantCommand(() -> mSwerveFieldOriented = !mSwerveFieldOriented));

    driverController
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        mDrive.setPose(
                            new Pose2d(
                                mDrive.getPose().getTranslation(),
                                new Rotation2d()
                                    .plus(
                                        ((DriverStation.getAlliance().orElse(Alliance.Blue)
                                                == Alliance.Red)
                                            ? Rotation2d.kPi
                                            : Rotation2d.kZero)))),
                    mDrive)
                .ignoringDisable(true));

    driverController
        .povUp()
        .whileTrue(
            DriveCommands.joystickDrive(
                mDrive, () -> 1, () -> 0, () -> 0, () -> mSwerveFieldOriented));

    driverController
        .povDown()
        .whileTrue(
            DriveCommands.joystickDrive(
                mDrive, () -> -1, () -> 0, () -> 0, () -> mSwerveFieldOriented));

    driverController
        .povLeft()
        .whileTrue(
            DriveCommands.joystickDrive(
                mDrive, () -> 0, () -> 0, () -> 1, () -> mSwerveFieldOriented));

    driverController
        .povRight()
        .whileTrue(
            DriveCommands.joystickDrive(
                mDrive, () -> 0, () -> 0, () -> -1, () -> mSwerveFieldOriented));

    driverController
        .a()
        .whileTrue(
            new GoToPose(
                () -> {
                  Pose2d curPose = mDrive.getPose();
                  Pose2d visionPose = mVision.getPoseInFrontOfAprilTag(21, 16);
                  Pose2d newPose =
                      new Pose2d(
                          visionPose.getTranslation(), new Rotation2d(Units.degreesToRadians(45)));
                  Logger.recordOutput("Debugging/Tuning Setpoint", newPose);
                  return newPose;
                },
                () -> mDrive.getPose(),
                mDrive));

    driverController
        .y()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> mDrive.setPose(new Pose2d(2.126, 2.180, Rotation2d.kZero))),
                new GoToPose(
                    () -> new Pose2d(1.678, 2.180, new Rotation2d(Units.degreesToRadians(0))),
                    () -> mDrive.getPose(),
                    mDrive),
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new WaitCommand(0.3),
                        mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE),
                        mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE)),
                    mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.INTAKING)),
                new GoToPose(
                    () -> new Pose2d(1.163, 2.180, new Rotation2d(Units.degreesToRadians(0))),
                    () -> mDrive.getPose(),
                    mDrive),
                new ParallelRaceGroup(
                        mIntake.setRollerCmd(IntakeConstants.IntakeRoller.kIntakeSpeed),
                        mIntake.setPIDIntakePivotCmd(
                            IntakeConstants.IntakePivot.Setpoints.INTAKING),
                        new SequentialCommandGroup(
                            mIntake.setIndexCoralCmd(),
                            new WaitCommand(0.1),
                            new ParallelCommandGroup(
                                mElevator.setPIDCmd(ElevatorConstants.Setpoints.POSTINTAKE),
                                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE),
                                mClaw.intakeCoralCmd()),
                            mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE)))
                    .andThen(new InstantCommand(() -> mIntake.setVoltsIntakeRoller(0)))));
  }

  public void initOperatorButtonboard() {

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kScoreCoral)
        .whileTrue(new DynamicCommand(() -> getScoreCmd(currentScoreLevel)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kClimbAscend)
        .whileTrue(
            new ParallelCommandGroup(
                mWrist.setPIDCmd(WristConstants.Setpoints.CLIMB).andThen(mWrist.enableFFCmd()),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.Climb),
                mClimb.climbToSetpoint(ClimbConstants.Pulley.Setpoints.CLIMBED),
                mClimb.setGrabberVoltsCmd(0.0),
                mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.STOWED)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kClimbDescend)
        .whileTrue(mClimb.climbToSetpoint(ClimbConstants.Pulley.Setpoints.EXTENDED));
    // .whileFalse();

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
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.L2ALGAE).andThen(mWrist.enableFFCmd()),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get())))
        // .onFalse(mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get()))
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> currentScoreLevel = 0),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE).andThen(mWrist.enableFFCmd()),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get())));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kAlgaePickupL3)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.L3ALGAE).andThen(mWrist.enableFFCmd()),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get())))
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> currentScoreLevel = 0),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE).andThen(mWrist.enableFFCmd()),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get())));
    // .onFalse(
    // new ParallelCommandGroup(
    // mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
    // mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
    // new InstantCommand(() ->
    // mClaw.setClaw(ClawConstants.RollerSpeed.HOLD_ALGAE))));
    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kPickup)
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
                    () ->
                        getScoreCmd(
                            5)))) // mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get()));
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
                IntakeConstants.IntakePivot.Setpoints.INTAKING.getPos()));
    // driverController.povLeft().whileTrue(mIntake.setTunablePivotCmd());

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
    // driverController.povDown().whileTrue(mElevator.setPIDCmd(ElevatorConstants.Setpoints.BOTTOM));
    // driverController.povLeft().whileTrue(mWrist.setTunablePIDCmd());
    // driverController.povRight().whileTrue(mWrist.setTunablePIDCmd());
    driverController.x().whileTrue(mElevator.setVoltsCmd(-2));
    driverController.b().whileTrue(mElevator.setVoltsCmd(2));
  }

  private Command getScoreCmd(int level) {
    int curLevel = MathUtil.clamp(level, 0, 5);

    if (curLevel == 1) {
      return mClaw.setClawCmd(ClawConstants.RollerSpeed.OUTTAKE_L1.get());
    } else if (curLevel == 2) {
      return new ParallelCommandGroup(
          mWrist.setPIDCmd(WristConstants.Setpoints.L2SCORE).andThen(mWrist.enableFFCmd()),
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
              // new WaitCommand(0.25).andThen(mClaw.setClawCmd(0.0)),
              new SequentialCommandGroup(
                  mElevator.setPIDCmd(ElevatorConstants.Setpoints.BARGE),
                  mWrist.setPIDCmd(WristConstants.Setpoints.THROW_ALGAE)),
              mClaw.throwAlgae(mWrist, mElevator)));
    } else {

      return new ParallelCommandGroup(
          mWrist.setPIDCmd(WristConstants.Setpoints.SCORE).andThen(mWrist.enableFFCmd()),
          new WaitCommand(0.1).andThen(mClaw.setClawCmd(-1.0)));
    }
  }
}
