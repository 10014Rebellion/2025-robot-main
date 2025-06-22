package frc.robot.subsystems.controllers;

import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ButtonBindings {
  private final Drive mDrive;
  private final ElevatorSubsystem mElevator;
  private final IntakeSubsystem mIntake;
  private final WristSubsystem mWrist;
  private final ClawSubsystem mClaw;
  private final ClimbSubsystem mClimb;

  public ButtonBindings(Drive pDrive, ElevatorSubsystem pElevator, IntakeSubsystem pIntake, WristSubsystem pWrist, ClawSubsystem pClaw, ClimbSubsystem pClimb){
    this.mDrive = pDrive;
    this.mElevator = pElevator;
    this.mIntake = pIntake;
    this.mWrist = pWrist;
    this.mClaw = pClaw;
    this.mClimb = pClimb;
  }
}
