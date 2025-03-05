// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawFFCommand;
import frc.robot.subsystems.claw.ClawIntakeCoralCommand;
import frc.robot.subsystems.claw.ClawPIDCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorFFCommand;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.intake.IntakeConstants.IntakePositions;
import frc.robot.subsystems.intake.IntakePIDCommand;
import frc.robot.subsystems.intake.OTBIntake;
import frc.robot.subsystems.intake.autoIntakeCoralCommand;
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
    SmartDashboard.putNumber(
        "TunableNumbers/Elevator/Tunable Setpoint", ElevatorConstants.Positions.L2.getPos());
    SmartDashboard.putNumber(
        "TunableNumbers/Wrist/Tunable Setpoint", ClawConstants.Wrist.Positions.L2.getPos());
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
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> manipulatorToWrist(levelSetpointInt)),
                new InstantCommand(() -> manipulatorToElevator(levelSetpointInt)),
                new SequentialCommandGroup(
                    new ClawPIDCommand(true, ClawConstants.Wrist.Positions.L2.getPos(), mClaw),
                    new ElevatorPIDCommand(
                        true, ElevatorConstants.Positions.L2.getPos(), mElevator),
                    new GoToPose(
                        () -> mVision.getReefScoringPose(7, 5, sideScoring),
                        () -> mVision.getPose(),
                        mDrive),
                    new WaitCommand(0.1),
                    new SequentialCommandGroup(
                        new ClawPIDCommand(ClawConstants.Wrist.Positions.SCORE, mClaw),
                        new ElevatorPIDCommand(ElevatorConstants.Positions.SCORE, mElevator)),
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw))))
        .onFalse(
            new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator)));

    driverController
        .leftTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> manipulatorToWrist(levelSetpointInt)),
                new InstantCommand(() -> manipulatorToElevator(levelSetpointInt)),
                new SequentialCommandGroup(
                    new ClawPIDCommand(true, ClawConstants.Wrist.Positions.L2.getPos(), mClaw),
                    new ElevatorPIDCommand(
                        true, ElevatorConstants.Positions.L2.getPos(), mElevator),
                    new WaitCommand(0.1),
                    new SequentialCommandGroup(
                        new ClawPIDCommand(ClawConstants.Wrist.Positions.SCORE, mClaw),
                        new ElevatorPIDCommand(ElevatorConstants.Positions.SCORE, mElevator)),
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw))))
        .onFalse(
            new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator)));
    driverController
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                new autoIntakeCoralCommand(mIntake),
                new SequentialCommandGroup(
                    new ElevatorPIDCommand((ElevatorConstants.Positions.PREINTAKE), mElevator),
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw))))
        .whileFalse(
            new ParallelCommandGroup(
                new IntakePIDCommand(IntakePositions.STOWED, mIntake),
                new InstantCommand(() -> mIntake.setFunnel(0)),
                new InstantCommand(() -> mIntake.setIndexer(0)),
                new InstantCommand(() -> mIntake.setRightRoller(0)),
                new InstantCommand(() -> mClaw.setClaw(0))));
    driverController
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, mElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw),
                new InstantCommand(
                    () -> mClaw.setClaw(ClawConstants.Claw.ClawRollerVolt.INTAKE_CORAL))))
        .whileFalse(new InstantCommand(() -> mClaw.setClaw(0)));

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
        .y()
        .whileTrue(
            new SequentialCommandGroup(
                new ClawPIDCommand(ClawConstants.Wrist.Positions.L3, mClaw),
                new ClawIntakeCoralCommand(mClaw)))
        .whileFalse(new ClawFFCommand(mClaw));
    // .whileTrue(
    //     new ParallelCommandGroup(
    //         new ElevatorPIDCommand(true, 15, mElevator),
    //         new InstantCommand(
    //             () -> mClaw.setClaw(ClawConstants.Claw.ClawRollerVolt.INTAKE_CORAL))))
    // .whileFalse(
    //     new ParallelCommandGroup(
    //         new ElevatorFFCommand(mElevator), new InstantCommand(() -> mClaw.setClaw(0))));
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

    operatorButtonboard
        .axisGreaterThan(1, 0.5)
        .whileTrue(new InstantCommand(() -> mElevator.setMotorVoltage(3)))
        .whileFalse(new ElevatorFFCommand(mElevator));

    operatorButtonboard
        .axisLessThan(1, -0.50)
        .whileTrue(new InstantCommand(() -> System.out.println("MOVING ELEVATOR DOWN")))
        .whileFalse(new ElevatorFFCommand(mElevator));
  }

  private void manipulatorToElevator(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);

    if (curLevel == 4) {
      SmartDashboard.putNumber(
          "TunableNumbers/Elevator/Tunable Setpoint", ElevatorConstants.Positions.L4.getPos());
    } else if (curLevel == 3) {
      SmartDashboard.putNumber(
          "TunableNumbers/Elevator/Tunable Setpoint", ElevatorConstants.Positions.L3.getPos());
    } else if (curLevel == 2) {
      SmartDashboard.putNumber(
          "TunableNumbers/Elevator/Tunable Setpoint", ElevatorConstants.Positions.L2.getPos());
    } else
      SmartDashboard.putNumber(
          "TunableNumbers/Elevator/Tunable Setpoint", ElevatorConstants.Positions.L1.getPos());
  }

  private void manipulatorToWrist(IntSupplier level) {
    int curLevel = MathUtil.clamp(level.getAsInt(), 0, 4);
    if (curLevel == 4) {
      SmartDashboard.putNumber(
          "TunableNumbers/Wrist/Setpoint", ClawConstants.Wrist.Positions.L4.getPos());
    } else if (curLevel == 3) {
      SmartDashboard.putNumber(
          "TunableNumbers/Wrist/Setpoint", ClawConstants.Wrist.Positions.L3.getPos());
    } else if (curLevel == 2) {
      SmartDashboard.putNumber(
          "TunableNumbers/Wrist/Setpoint", ClawConstants.Wrist.Positions.L2.getPos());
    } else
      SmartDashboard.putNumber(
          "TunableNumbers/Wrist/Setpoint", ClawConstants.Wrist.Positions.L1.getPos());
  }
}
