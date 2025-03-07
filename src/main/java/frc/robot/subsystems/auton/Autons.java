package frc.robot.subsystems.auton;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.subsystems.elevatorPivot.ElevatorPivot;
import frc.robot.subsystems.intake.OTBIntake;
import frc.robot.subsystems.intake.autoIntakeCoralCommand;

public class Autons {
  private final Drive mDrive;
  private final Claw mClaw;
  private final Elevator mElevator;
  private final ElevatorPivot mPivot;
  private final OTBIntake mIntake;

  public Autons(
      Drive pDrive, Claw pClaw, Elevator pElevator, ElevatorPivot pPivot, OTBIntake pIntake) {
    this.mDrive = pDrive;
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
  }

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
}
