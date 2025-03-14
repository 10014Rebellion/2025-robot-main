// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

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
  private final PivotSubsystem mPivot;

  public ControlsSubsystem(
      DriveSubsystem pDrive,
      VisionSubsystem pVision,
      WristSubsystem pWrist,
      ElevatorSubsystem pElevator,
      PivotSubsystem pPivot,
      IntakeSubsystem pIntake,
      ClawSubsystem pClaw) {
    this.mDrive = pDrive;
    this.mVision = pVision;
    this.mWrist = pWrist;
    this.mElevator = pElevator;
    this.mPivot = pPivot;
    this.mIntake = pIntake;
    this.mClaw = pClaw;

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
        .rightTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> levelToDrivebase(levelSetpointInt)),
                new GoToPose(
                    () ->
                        mVision.getClosestReefScoringPose(
                            distanceScoring, () -> VisionConstants.PoseOffsets.RIGHT),
                    () -> mDrive.getPose(),
                    mDrive)));

    driverController
        .leftTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> levelToDrivebase(levelSetpointInt)),
                new GoToPose(
                    () ->
                        mVision.getClosestReefScoringPose(
                            distanceScoring, () -> VisionConstants.PoseOffsets.LEFT),
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

  public void initOperatorButtonboard() {
    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kReadyScoring)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelToWrist(levelSetpointInt)),
                new InstantCommand(() -> levelToElevator(levelSetpointInt)),
                new SequentialCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2),
                    mWrist.setPIDCmd(WristConstants.Setpoints.L2))));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kScoreCoral)
        .whileTrue(
            new ParallelCommandGroup(
                mWrist.setPIDCmd(WristConstants.Setpoints.SCORE),
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.SCORE)))
        .onFalse(mWrist.setPIDCmd(WristConstants.Setpoints.INTAKE));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kClimbPullUp)
        .whileTrue(
            new ParallelCommandGroup(
                mPivot.setVoltsCmd(12), mWrist.setPIDCmd(WristConstants.Setpoints.CLIMB)))
        .onFalse(mPivot.stopCommand());

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kClimbLetGo)
        .whileTrue(mPivot.setVoltsCmd(-12))
        .onFalse(mPivot.stopCommand());

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kSetScoreL4)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelSetpointInt = () -> 4),
                new SequentialCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.L4),
                    mWrist.setPIDCmd(WristConstants.Setpoints.L4))));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kSetScoreL3)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelSetpointInt = () -> 3),
                new SequentialCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3),
                    mWrist.setPIDCmd(WristConstants.Setpoints.L3))));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kSetScoreL2)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelSetpointInt = () -> 2),
                new SequentialCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2)
                    // mWrist.setPIDCmd(WristConstants.Setpoints.L2)
                    )));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kSetScoreL1)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelSetpointInt = () -> 1),
                new SequentialCommandGroup(
                    mElevator.setPIDCmd(ElevatorConstants.Setpoints.L1)
                    // mWrist.setPIDCmd(WristConstants.Setpoints.L1)
                    )));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kAlgaePickupL2)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L2ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.L2ALGAE),
                new InstantCommand(() -> mClaw.setClaw(ClawConstants.Setpoints.INTAKE_ALGAE))))
        .onFalse(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
                new InstantCommand(() -> mClaw.setClaw(ClawConstants.Setpoints.HOLD_ALGAE))));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kAlgaePickupL3)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.L3ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.L3ALGAE),
                new InstantCommand(() -> mClaw.setClaw(ClawConstants.Setpoints.INTAKE_ALGAE))))
        .onFalse(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.HOLD_ALGAE),
                mWrist.setPIDCmd(WristConstants.Setpoints.HOLD_ALGAE),
                new InstantCommand(() -> mClaw.setClaw(ClawConstants.Setpoints.HOLD_ALGAE))));

    operatorButtonboard.axisLessThan(1, -0.5).whileTrue(mElevator.setVoltsCmd(3));

    operatorButtonboard.axisGreaterThan(1, 0.5).whileTrue(mElevator.setVoltsCmd(-3));

    operatorButtonboard.axisGreaterThan(0, 0.5).whileTrue(mWrist.setVoltsCmd(2));

    operatorButtonboard.axisLessThan(0, -0.50).whileTrue(mWrist.setVoltsCmd(-2));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kEjectAlgaeToBarge)
        .whileTrue(mClaw.setClawCmd(ClawConstants.Setpoints.OUTTAKE_BARGE.get()))
        .whileFalse(new InstantCommand(() -> mClaw.setClaw(0)));

    operatorButtonboard
        .button(ControlsConstants.Buttonboard.kGoToBarge)
        .whileTrue(
            new ParallelCommandGroup(
                mElevator.setPIDCmd(ElevatorConstants.Setpoints.BARGE),
                mWrist.setPIDCmd(WristConstants.Setpoints.BARGE)));
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
