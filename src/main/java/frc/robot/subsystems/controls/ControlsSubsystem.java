// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ControlsSubsystem extends SubsystemBase {
  private final CommandXboxController driverController = new CommandXboxController(0);
  // private final CommandXboxController operatorController = new
  // CommandXboxController(1);
  private final CommandGenericHID operatorButtonboard = new CommandGenericHID(1);

  private IntSupplier levelSetpointInt;
  private Supplier<VisionConstants.PoseOffsets> sideScoring;
  private Supplier<VisionConstants.linearPoseOffsets> distanceScoring;
  private boolean mSwerveFieldOriented = true;

  // private final double kDriveSwerveMultipler = 0.5;
  // private final double kRotationSwerveMultipler = 0.6;

  private final DriveSubsystem mDrive;
  private final VisionSubsystem mVision;
  private final WristSubsystem mWrist;
  private final ElevatorSubsystem mElevator;
  private final IntakeSubsystem mIntake;
  private final ClawSubsystem mClaw;
  private final ClimbSubsystem mClimb;

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

    levelSetpointInt = () -> 2;
    sideScoring = () -> VisionConstants.PoseOffsets.LEFT;
    distanceScoring = () -> VisionConstants.linearPoseOffsets.L2;
    SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Setpoints.L2.getPos());
    SmartDashboard.putNumber("Levels/Wrist Setpoint", WristConstants.Setpoints.L2.getPos());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Levels/Chosen Level", levelSetpointInt.getAsInt());
    SmartDashboard.putBoolean(
        "Levels/Left Side Chosen", sideScoring.get().equals(VisionConstants.PoseOffsets.LEFT));
    SmartDashboard.putNumber("Levels/Distance Offset", distanceScoring.get().getOffsetM());
  }

  public void initDriverController() {
    // driverController
    //     .povUp()
    //     .whileTrue(
    //         new InstantCommand(
    //             () ->
    //                 mClimb.setGrabberVolts(
    //                     ClimbConstants.Grabber.VoltageSetpoints.PULL_IN.getVolts())));
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
                mWrist.setPIDCmd(WristConstants.Setpoints.GROUNDINTAKE),
                new ParallelCommandGroup(
                    mClaw.intakeCoralCmd(),
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.GROUNDINTAKE))));
    driverController.a().whileTrue(mClaw.intakeCoralCmd());
  }

  public void initDrivebase() {
    mDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            mDrive,
            () ->
                -Math.pow(driverController.getLeftY(), 2)
                    * Math.signum(driverController.getLeftY()),
            () ->
                -Math.pow(driverController.getLeftX(), 2)
                    * Math.signum(driverController.getLeftX()),
            () ->
                -Math.pow(driverController.getRightX(), 2)
                    * Math.signum(driverController.getRightX()),
            () -> mSwerveFieldOriented));

    driverController
        .b()
        .onTrue(new InstantCommand(() -> mSwerveFieldOriented = !mSwerveFieldOriented));

    driverController
        .rightTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> levelToDrivebase(levelSetpointInt)),
                new GoToPose(
                    () -> mVision.getClosestReefScoringPose(VisionConstants.PoseOffsets.RIGHT),
                    () -> mDrive.getPose(),
                    mDrive)));

    driverController
        .leftTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new GoToPose(
                    () -> mVision.getClosestReefScoringPose(VisionConstants.PoseOffsets.LEFT),
                    () -> mDrive.getPose(),
                    mDrive)));

    driverController
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        mDrive.setPose(
                            new Pose2d(mDrive.getPose().getTranslation(), new Rotation2d())),
                    mDrive)
                .ignoringDisable(true));
  }

  public void initTuningDrive() {
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
                            new Pose2d(mDrive.getPose().getTranslation(), new Rotation2d())),
                    mDrive)
                .ignoringDisable(true));

    driverController
        .povUp()
        .whileTrue(
            DriveCommands.joystickDrive(
                mDrive, () -> -1, () -> 0, () -> 0, () -> mSwerveFieldOriented));

    driverController
        .povDown()
        .whileTrue(
            DriveCommands.joystickDrive(
                mDrive, () -> 1, () -> 0, () -> 0, () -> mSwerveFieldOriented));

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
    // operatorButtonboard
    //     .button(ControlsConstants.Buttonboard.kReadyScoring)
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             new InstantCommand(() -> levelToWrist(levelSetpointInt)),
    //             new InstantCommand(() -> levelToElevator(levelSetpointInt)),
    //             new SequentialCommandGroup(
    //                 mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2),
    //                 mWrist.setPIDCmd(WristConstants.Setpoints.L2))));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kScoreCoral)
        .whileTrue(
            new SequentialCommandGroup(
                mWrist.setPIDCmd(WristConstants.Setpoints.SCORE), mClaw.setClawCmd(-0.5)));

    // operatorButtonboard
    //     .button(ControlsConstants.Buttonboard.kClimbAscend)
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             mWrist.setPIDCmd(WristConstants.Setpoints.REVERSEL4),
    //             mElevator.setPIDCmd(ElevatorConstants.Setpoints.ReverseL4)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kClimbAscend)
        .whileTrue(
            // new ParallelCommandGroup(
            //     mWrist.setPIDCmd(WristConstants.Setpoints.REVERSEL4),
            //     mElevator.setPIDCmd(ElevatorConstants.Setpoints.REVERSESCORE),
            //     new WaitCommand(0.2)
            //         .andThen(mClaw.scoreCoralCmd(ClawConstants.RollerSpeed.REVERSE_REEF))));
            // mClimb.setPulleyVoltsCmd(ClimbConstants.Pulley.VoltageSetpoints.DESCEND));
            new ParallelCommandGroup(
                mWrist.setPIDCmd(WristConstants.Setpoints.CLIMB),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.Climb),
                mClimb.setPulleyVoltsCmd(ClimbConstants.Pulley.VoltageSetpoints.ASCEND),
                mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.STOWED)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kClimbDescend)
        .whileTrue(mClimb.setPulleyVoltsCmd(ClimbConstants.Pulley.VoltageSetpoints.DESCEND));
    // .whileFalse();

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kSetScoreL4)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L4),
                mWrist.setPIDCmd(WristConstants.Setpoints.L4)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kSetScoreL3)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3),
                mWrist.setPIDCmd(WristConstants.Setpoints.L3)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kSetScoreL2)
        .whileTrue(
            new SequentialCommandGroup(
                mWrist.setPIDCmd(WristConstants.Setpoints.L2),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kSetScoreL1)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L1),
                mWrist.setPIDCmd(WristConstants.Setpoints.L1)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kAlgaePickupL2)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.L2ALGAE),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get())))
        .onFalse(mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get()))
        .onFalse(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get())));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kAlgaePickupL3)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.L3ALGAE),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get())))
        .onFalse(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
                mClaw.setClawCmd(ClawConstants.RollerSpeed.HOLD_ALGAE.get())));
    // .onFalse(
    //     new ParallelCommandGroup(
    //         mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
    //         mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
    //         new InstantCommand(() -> mClaw.setClaw(ClawConstants.RollerSpeed.HOLD_ALGAE))));
    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kPickup)
        .whileTrue(
            new SequentialCommandGroup(
                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE),
                new ParallelCommandGroup(
                    mClaw.intakeCoralCmd(),
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.POSTINTAKE))))
        .whileFalse(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.PREINTAKE),
                mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE)));

    operatorButtonboard.axisLessThan(1, -0.5).whileTrue(mElevator.setVoltsCmd(3));

    operatorButtonboard.axisGreaterThan(1, 0.5).whileTrue(mElevator.setVoltsCmd(-3));

    operatorButtonboard.axisGreaterThan(0, 0.5).whileTrue(mWrist.setVoltsCmd(2));

    operatorButtonboard.axisLessThan(0, -0.50).whileTrue(mWrist.setVoltsCmd(-2));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kEjectAlgaeToBarge)
        .whileTrue(mClaw.setClawCmd(ClawConstants.RollerSpeed.OUTTAKE_L1.get()))
        .whileFalse(new InstantCommand(() -> mClaw.setClaw(0)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kGoToBarge)
        .whileTrue(mClaw.setClawCmd(ClawConstants.RollerSpeed.INTAKE_ALGAE.get()));
  }

  public void initIntakeTuning() {
    driverController.povRight().whileTrue(mIntake.setTunablePIDIntakeCommand());
    driverController
        .povLeft()
        .whileTrue(mIntake.setPIDIntakePivotCmd(IntakeConstants.IntakePivot.Setpoints.INTAKING));
    // driverController.povLeft().whileTrue(mIntake.setTunablePivotCmd());

    driverController.povUp().whileTrue(mIntake.setPivotCmd(3.0));
    driverController.povDown().whileTrue(mIntake.setPivotCmd(-1));

    driverController.rightBumper().whileTrue(mIntake.setRollerCmd(6.0));
    driverController.leftBumper().whileTrue(mIntake.setRollerCmd(-6.0));
  }

  public void initElevatorTuning() {
    driverController
        .axisMagnitudeGreaterThan(1, 0.2)
        .whileTrue(mElevator.setVoltsCmd(driverController.getLeftY() * 8));
    driverController
        .povUp()
        .whileTrue(mElevator.setVoltsCmd(12)); // mElevator.setTunablePIDCommand());
    driverController.povDown().whileTrue(mElevator.setPIDCmd(ElevatorConstants.Setpoints.BOTTOM));
    driverController.povLeft().whileTrue(mWrist.setVoltsCmd(2));
    driverController.povRight().whileTrue(mWrist.setVoltsCmd(-2));
  }

  private void levelToElevator(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);
    // this is atrocious please redo this to work with suppliers ;-;
    // That's week 4 tho not just yet (3/4/2025)
    if (curLevel == 4) {
      SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Setpoints.L4.getPos());
    } else if (curLevel == 3) {
      SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Setpoints.L3.getPos());
    } else if (curLevel == 2) {
      SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Setpoints.L2.getPos());
    } else
      SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Setpoints.L1.getPos());
  }

  private void levelToWrist(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);
    if (curLevel == 4) {
      SmartDashboard.putNumber("Levels/Wrist Setpoint", WristConstants.Setpoints.L4.getPos());
    } else if (curLevel == 3) {
      SmartDashboard.putNumber("Levels/Wrist Setpoint", WristConstants.Setpoints.L3.getPos());
    } else if (curLevel == 2) {
      SmartDashboard.putNumber("Levels/Wrist Setpoint", WristConstants.Setpoints.L2.getPos());
    } else SmartDashboard.putNumber("Levels/Wrist Setpoint", WristConstants.Setpoints.L1.getPos());
  }

  private void levelToDrivebase(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);
    if (curLevel == 4) {
      distanceScoring = () -> VisionConstants.linearPoseOffsets.L4;
    } else if (curLevel == 3) {
      distanceScoring = () -> VisionConstants.linearPoseOffsets.L3;
    } else if (curLevel == 2) {
      distanceScoring = () -> VisionConstants.linearPoseOffsets.L2;
    } else distanceScoring = () -> VisionConstants.linearPoseOffsets.L2;
  }
}
