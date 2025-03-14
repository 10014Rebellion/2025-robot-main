// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawManualCommand;
import frc.robot.subsystems.claw.NewClawPIDCommand;
import frc.robot.subsystems.claw.tempClaw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorFFCommand;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.elevator.elevatorManualCommand;
import frc.robot.subsystems.elevatorPivot.ElevatorPivot;
import frc.robot.subsystems.intake.OTBIntake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class Controllers extends SubsystemBase {
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

  private final Drive mDrive;
  private final Vision mVision;
  private final Elevator mElevator;
  private final OTBIntake mIntake;
  private final Claw mClaw;
  private final ElevatorPivot mPivot;
  private final tempClaw mNewClaw;

  public Controllers(
      Drive pDrive,
      Vision pVision,
      Elevator pElevator,
      ElevatorPivot pPivot,
      OTBIntake pIntake,
      Claw pClaw,
      tempClaw pTempClaw) {
    this.mDrive = pDrive;
    this.mVision = pVision;
    this.mElevator = pElevator;
    this.mPivot = pPivot;
    this.mIntake = pIntake;
    this.mClaw = pClaw;
    this.mNewClaw = pTempClaw;

    levelSetpointInt = () -> 2;
    sideScoring = () -> VisionConstants.PoseOffsets.LEFT;
    distanceScoring = () -> VisionConstants.linearPoseOffsets.L2;
    SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Positions.L2.getPos());
    SmartDashboard.putNumber("Levels/Wrist Setpoint", ClawConstants.Wrist.Positions.L2.getPos());
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
        .x()
        .onTrue(
            new InstantCommand(() -> mDrive.setPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)))));
  }

  public void initOperatorButtonboard() {
    operatorButtonboard.axisGreaterThan(0, 0.5).whileTrue(new ClawManualCommand(mNewClaw, 1));
    operatorButtonboard.axisLessThan(0, -0.5).whileTrue(new ClawManualCommand(mNewClaw, -1));
    operatorButtonboard
        .axisGreaterThan(1, 0.5)
        .whileTrue(new elevatorManualCommand(mElevator, 3))
        .whileFalse(new ElevatorFFCommand(mElevator));
    operatorButtonboard
        .axisLessThan(1, -0.5)
        .whileTrue(new elevatorManualCommand(mElevator, -3))
        .whileFalse(new ElevatorFFCommand(mElevator));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetScoreL4)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L4, mElevator),
                new NewClawPIDCommand(ClawConstants.Wrist.Positions.L4.getPos(), mNewClaw)));
    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetScoreL3)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L3, mElevator),
                new NewClawPIDCommand(ClawConstants.Wrist.Positions.L3.getPos(), mNewClaw)));
    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetScoreL2)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L2, mElevator),
                new NewClawPIDCommand(ClawConstants.Wrist.Positions.L2.getPos(), mNewClaw)));
    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetScoreL1)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L1, mElevator),
                new NewClawPIDCommand(ClawConstants.Wrist.Positions.L1.getPos(), mNewClaw)));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kScoreCoral)
        .onTrue(
            new SequentialCommandGroup(
                new NewClawPIDCommand(ClawConstants.Wrist.Positions.SCORE.getPos(), mNewClaw),
                new InstantCommand(() -> mNewClaw.setClaw(-2))))
        .onFalse(new InstantCommand(() -> mNewClaw.setClaw(0)));
    // new SequentialCommandGroup(
    //   new WaitCommand(0.2),
    //   new InstantCommand(() -> mNewClaw.setClaw(-2))
    // );
  }

  public void initTestingController() {
    driverController
        .povUp()
        .whileTrue(
            new ParallelCommandGroup(
                new NewClawPIDCommand(45.0, mNewClaw),
                new ElevatorPIDCommand(ElevatorConstants.Positions.L3, mElevator)))
        .whileFalse(new ElevatorFFCommand(mElevator));
    driverController
        .povRight()
        .whileTrue(
            new ParallelCommandGroup(
                new NewClawPIDCommand(0.0, mNewClaw),
                new ElevatorPIDCommand(ElevatorConstants.Positions.L3, mElevator)))
        .whileFalse(new ElevatorFFCommand(mElevator));
    driverController
        .povDown()
        .whileTrue(
            new ParallelCommandGroup(
                new NewClawPIDCommand(-45.0, mNewClaw),
                new ElevatorPIDCommand(ElevatorConstants.Positions.L3, mElevator)))
        .whileFalse(new ElevatorFFCommand(mElevator));
    driverController
        .y()
        .whileTrue(new InstantCommand(() -> mNewClaw.setWrist(2)))
        .whileFalse(new InstantCommand(() -> mNewClaw.setWrist(0)));
    driverController
        .a()
        .whileTrue(new InstantCommand(() -> mNewClaw.setWrist(-2)))
        .whileFalse(new InstantCommand(() -> mNewClaw.setWrist(0)));
    driverController
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.GROUNDINTAKE, mElevator),
                new NewClawPIDCommand(
                    ClawConstants.Wrist.Positions.GROUNDINTAKE.getPos(), mNewClaw),
                new InstantCommand(() -> mNewClaw.setClaw(5))))
        .whileFalse(new InstantCommand(() -> mNewClaw.setClaw(0)));
    driverController
        .leftBumper()
        .whileTrue(new InstantCommand(() -> mNewClaw.setClaw(-2)))
        .whileFalse(new InstantCommand(() -> mNewClaw.setClaw(0)));
  }

  private void levelToElevator(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);
    // this is atrocious please redo this to work with suppliers ;-;
    // That's week 4 tho not just yet (3/4/2025)
    if (curLevel == 4) {
      SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Positions.L4.getPos());
    } else if (curLevel == 3) {
      SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Positions.L3.getPos());
    } else if (curLevel == 2) {
      SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Positions.L2.getPos());
    } else
      SmartDashboard.putNumber("Levels/Elevator Setpoint", ElevatorConstants.Positions.L1.getPos());
  }

  private void levelToWrist(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);
    if (curLevel == 4) {
      SmartDashboard.putNumber("Levels/Wrist Setpoint", ClawConstants.Wrist.Positions.L4.getPos());
    } else if (curLevel == 3) {
      SmartDashboard.putNumber("Levels/Wrist Setpoint", ClawConstants.Wrist.Positions.L3.getPos());
    } else if (curLevel == 2) {
      SmartDashboard.putNumber("Levels/Wrist Setpoint", ClawConstants.Wrist.Positions.L2.getPos());
    } else
      SmartDashboard.putNumber("Levels/Wrist Setpoint", ClawConstants.Wrist.Positions.L1.getPos());
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
