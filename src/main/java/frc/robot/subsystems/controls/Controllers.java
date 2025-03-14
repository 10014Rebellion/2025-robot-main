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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;
import frc.robot.subsystems.claw.ClawFFCommand;
import frc.robot.subsystems.claw.ClawIntakeCoralCommand;
import frc.robot.subsystems.claw.ClawLevelPIDCommand;
import frc.robot.subsystems.claw.ClawManualCommand;
import frc.robot.subsystems.claw.ClawPIDCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorFFCommand;
import frc.robot.subsystems.elevator.ElevatorLevelPIDCommand;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.elevator.elevatorManualCommand;
import frc.robot.subsystems.intake.IntakeConstants.IntakePositions;
import frc.robot.subsystems.pivot.ElevatorPivot;
import frc.robot.subsystems.intake.IntakePIDCommand;
import frc.robot.subsystems.intake.OTBIntake;
import frc.robot.subsystems.intake.autoIntakeCoralCommand;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.wrist.WristConstants;

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

  public Controllers(
      Drive pDrive,
      Vision pVision,
      Elevator pElevator,
      ElevatorPivot pPivot,
      OTBIntake pIntake,
      Claw pClaw) {
    this.mDrive = pDrive;
    this.mVision = pVision;
    this.mElevator = pElevator;
    this.mPivot = pPivot;
    this.mIntake = pIntake;
    this.mClaw = pClaw;

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
        .rightBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new autoIntakeCoralCommand(mIntake),
                    new SequentialCommandGroup(
                        new ElevatorPIDCommand((ElevatorConstants.Positions.PREINTAKE), mElevator),
                        new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw))),
                new SequentialCommandGroup(
                    new ElevatorPIDCommand((ElevatorConstants.Positions.PREINTAKE), mElevator),
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw)),
                new ParallelCommandGroup(
                    new InstantCommand(() -> mIntake.setFunnel(2)),
                    new ClawIntakeCoralCommand(mClaw),
                    new SequentialCommandGroup(
                        new ElevatorPIDCommand((ElevatorConstants.Positions.POSTINTAKE), mElevator),
                        new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw)))))
        .whileFalse(
            new ParallelCommandGroup(
                new IntakePIDCommand(IntakePositions.STOWED, mIntake),
                new InstantCommand(() -> mIntake.setFunnel(0)),
                new InstantCommand(() -> mIntake.setIndexer(0)),
                new InstantCommand(() -> mIntake.setRightRoller(0)),
                new InstantCommand(() -> mClaw.setClaw(0))));
    // driverController
    // .rightBumper()
    // .whileTrue(
    // new SequentialCommandGroup(
    // new ClawPIDCommand(ClawConstants.Wrist.Positions.L2, mClaw),
    // new ClawIntakeCoralCommand(mClaw)))
    // .whileFalse(
    // new ParallelCommandGroup(
    // new ClawFFCommand(mClaw), new InstantCommand(() -> mClaw.setClaw(0))));
    driverController
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> ClawConstants.Claw.hasCoral = false),
                new IntakePIDCommand(IntakePositions.ALGAEINTAKE, mIntake),
                new InstantCommand(() -> mClaw.setClaw(-1)),
                new InstantCommand(() -> mIntake.setFunnel(-1)),
                new InstantCommand(() -> mIntake.setIndexer(-1)),
                new InstantCommand(() -> mIntake.setRightRoller(-2))))
        .whileFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> mClaw.setClaw(0)),
                new InstantCommand(() -> mIntake.setFunnel(0)),
                new InstantCommand(() -> mIntake.setIndexer(0)),
                new InstantCommand(() -> mIntake.setRightRoller(0))));
    // driverController
    // .b()
    // .whileTrue(
    // new ParallelCommandGroup(
    // new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, mElevator),
    // new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw),
    // new InstantCommand(
    // () -> mClaw.setClaw(ClawConstants.Claw.ClawRollerVolt.INTAKE_CORAL))))
    // .whileFalse(new InstantCommand(() -> mClaw.setClaw(0)));

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
                new ClawPIDCommand(WristConstants.Setpoints.L3, mClaw),
                new ClawIntakeCoralCommand(mClaw)))
        .whileFalse(new ClawFFCommand(mClaw));

    driverController
        .a()
        .whileTrue(new InstantCommand(() -> mClaw.setClaw(1)))
        .whileFalse(new InstantCommand(() -> mClaw.setClaw(0)));
  }

  public void initOperatorButtonboard() {
    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kReadyScoring)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelToWrist(levelSetpointInt)),
                new InstantCommand(() -> levelToElevator(levelSetpointInt)),
                new SequentialCommandGroup(
                    new ElevatorLevelPIDCommand(mElevator), new ClawLevelPIDCommand(mClaw))))
        .onFalse(
            new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator)));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kScoreCoral)
        .whileTrue(
            new ParallelCommandGroup(
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.SCORE, mClaw),
                    new ElevatorPIDCommand(ElevatorConstants.Positions.SCORE, mElevator))
                .andThen(new InstantCommand(() -> ClawConstants.Claw.hasCoral = false)))
        .onFalse(
            new SequentialCommandGroup(
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, mClaw),
                new ParallelCommandGroup(
                    new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator))));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kClimbPullUp)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> mPivot.setVoltage(12)),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.CLIMB, mClaw)))
        .onFalse(new ParallelCommandGroup(mPivot.stopCommand(), new ClawFFCommand(mClaw)));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kClimbLetGo)
        .whileTrue(new InstantCommand(() -> mPivot.setVoltage(-12)))
        .onFalse(mPivot.stopCommand());

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetScoreL4)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelSetpointInt = () -> 4),
                new SequentialCommandGroup(
                    new ElevatorPIDCommand(ElevatorConstants.Positions.L4, mElevator),
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.L4, mClaw))))
        .onFalse(
            new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator)));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetScoreL3)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelSetpointInt = () -> 3),
                new SequentialCommandGroup(
                    new ElevatorPIDCommand(ElevatorConstants.Positions.L3, mElevator),
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.L3, mClaw))))
        .onFalse(
            new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator)));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetScoreL2)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelSetpointInt = () -> 2),
                new SequentialCommandGroup(
                    new ElevatorPIDCommand(ElevatorConstants.Positions.L2, mElevator),
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.L2, mClaw))))
        .onFalse(
            new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator)));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kSetScoreL1)
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> levelSetpointInt = () -> 1),
                new SequentialCommandGroup(
                    new ElevatorPIDCommand(ElevatorConstants.Positions.L1, mElevator),
                    new ClawPIDCommand(ClawConstants.Wrist.Positions.L1, mClaw))))
        .onFalse(
            new ParallelCommandGroup(new ClawFFCommand(mClaw), new ElevatorFFCommand(mElevator)));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kAlgaePickupL2)
        .whileTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L2ALGAE, mElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.L2ALGAE, mClaw),
                new InstantCommand(() -> mClaw.setClaw(ClawRollerVolt.INTAKE_ALGAE))))
        .onFalse(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.HOLD_ALGAE, mElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.HOLD_ALGAE, mClaw),
                new InstantCommand(() -> mClaw.setClaw(ClawRollerVolt.HOLD_ALGAE))));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kAlgaePickupL3)
        .whileTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L3ALGAE, mElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.L3ALGAE, mClaw),
                new InstantCommand(() -> mClaw.setClaw(ClawRollerVolt.INTAKE_ALGAE))))
        .onFalse(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.HOLD_ALGAE, mElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.HOLD_ALGAE, mClaw),
                new InstantCommand(() -> mClaw.setClaw(ClawRollerVolt.HOLD_ALGAE))));

    operatorButtonboard
        .axisGreaterThan(1, 0.5)
        .onTrue(new elevatorManualCommand(mElevator, 3))
        .onFalse(new ElevatorFFCommand(mElevator));

    operatorButtonboard
        .axisLessThan(1, -0.50)
        .onTrue(new elevatorManualCommand(mElevator, -3))
        .onFalse(new ElevatorFFCommand(mElevator));

    operatorButtonboard
        .axisGreaterThan(0, 0.5)
        .onTrue(new ClawManualCommand(mClaw, 2))
        .onFalse(new ClawFFCommand(mClaw));

    operatorButtonboard
        .axisLessThan(0, -0.50)
        .onTrue(new ClawManualCommand(mClaw, -2))
        .onFalse(new ClawFFCommand(mClaw));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kEjectAlgaeToBarge)
        .whileTrue(
            new InstantCommand(
                () -> mClaw.setClaw(ClawConstants.Claw.ClawRollerVolt.OUTTAKE_BARGE)))
        .whileFalse(new InstantCommand(() -> mClaw.setClaw(0)));

    operatorButtonboard
        .button(ControllerConstants.Buttonboard.kGoToBarge)
        .whileTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.BARGE, mElevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.BARGE, mClaw)))
        .whileFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(mElevator), new ClawFFCommand(mClaw)));
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
