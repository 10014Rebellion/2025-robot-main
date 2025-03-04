// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToPose;
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawFFCommand;
import frc.robot.subsystems.claw.ClawPIDCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorFFCommand;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.intake.IntakeConstants.IntakePositions;
import frc.robot.subsystems.intake.IntakePIDCommand;
import frc.robot.subsystems.intake.OTBIntake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class Controllers extends SubsystemBase {
  private final CommandXboxController driverController = new CommandXboxController(0);
  // private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandGenericHID operatorButtonboard = new CommandGenericHID(2);

  private IntSupplier levelSetpointInt;
  private Supplier<VisionConstants.PoseOffsets> sideScoring;
  private boolean mSwerveFieldOriented = true;

  // private final double kDriveSwerveMultipler = 0.5;
  // private final double kRotationSwerveMultipler = 0.6;

  private final Drive mDrive;
  private final Vision mVision;
  private final Elevator mElevator;
  private final OTBIntake mIntake;
  private final Claw mClaw;

  public Controllers(
      Drive pDrive, Vision pVision, Elevator pElevator, OTBIntake pIntake, Claw pClaw) {
    this.mDrive = pDrive;
    this.mVision = pVision;
    this.mElevator = pElevator;
    this.mIntake = pIntake;
    this.mClaw = pClaw;

    levelSetpointInt = () -> 2;
    sideScoring = () -> VisionConstants.PoseOffsets.LEFT;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Chosen Level", levelSetpointInt.getAsInt());
    SmartDashboard.putBoolean(
        "Left Side Chosen", sideScoring.get().equals(VisionConstants.PoseOffsets.LEFT));
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
        .leftTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new ClawPIDCommand(manipulatorToWrist(levelSetpointInt), mClaw),
                new ElevatorPIDCommand(manipulatorToElevator(levelSetpointInt), mElevator),
                new GoToPose(
                    () -> mVision.getReefScoringPose(7, 0, sideScoring),
                    () -> mVision.getPose(),
                    mDrive),
                new WaitCommand(0.1),
                new ParallelCommandGroup(
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.SCORE, mClaw),
                    new ElevatorPIDCommand(ElevatorConstants.Positions.SCORE, mElevator)),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw)))
        .onFalse(
            new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator)));

    driverController
        .rightTrigger()
        .whileTrue(
            new GoToPose(
                () -> mVision.getReefScoringPose(7, 15, sideScoring),
                () -> mVision.getPose(),
                mDrive));
    driverController
        .rightBumper()
        .whileTrue(new IntakeCoral(mElevator, mClaw, mIntake))
        .whileFalse(
            new ParallelCommandGroup(
                new IntakePIDCommand(IntakePositions.STOWED, mIntake),
                new InstantCommand(() -> mIntake.setFunnel(0)),
                new InstantCommand(() -> mIntake.setIndexer(0)),
                new InstantCommand(() -> mIntake.setRightRoller(0))));
  }

  public void initOperatorButtonboard() {
    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetLeftPose)
        .whileTrue(new InstantCommand(() -> sideScoring = () -> VisionConstants.PoseOffsets.LEFT));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetRightPose)
        .whileTrue(new InstantCommand(() -> sideScoring = () -> VisionConstants.PoseOffsets.RIGHT));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kScoreL4)
        .whileTrue(new InstantCommand(() -> levelSetpointInt = () -> 4));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kScoreL3)
        .whileTrue(new InstantCommand(() -> levelSetpointInt = () -> 3));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kScoreL2)
        .whileTrue(new InstantCommand(() -> levelSetpointInt = () -> 2));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kScoreL1)
        .whileTrue(new InstantCommand(() -> levelSetpointInt = () -> 1));
  }

  private Supplier<ClawConstants.Wrist.Positions> manipulatorToWrist(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);

    if (curLevel == 4) {
      return () -> ClawConstants.Wrist.Positions.L4;
    }

    if (curLevel == 3) {
      return () -> ClawConstants.Wrist.Positions.L3;
    }

    if (curLevel == 2) {
      return () -> ClawConstants.Wrist.Positions.L2;
    }

    return () -> ClawConstants.Wrist.Positions.L1;
  }

  private Supplier<ElevatorConstants.Positions> manipulatorToElevator(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);
    if (curLevel == 4) {
      return () -> ElevatorConstants.Positions.L4;
    }

    if (curLevel == 3) {
      return () -> ElevatorConstants.Positions.L3;
    }

    if (curLevel == 2) {
      return () -> ElevatorConstants.Positions.L2;
    }
    return () -> ElevatorConstants.Positions.L1;
  }
}
