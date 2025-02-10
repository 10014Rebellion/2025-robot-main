// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Wrist;

public class ClawPID extends Command {
  private final Claw mClawSubsystem;
  private final double mSetpoint;
  private final ProfiledPIDController mProfiledPIDController;

  public ClawPID(double pSetpoint, Claw pClawSubsystem) {
    this.mClawSubsystem = pClawSubsystem;
    this.mSetpoint = MathUtil.clamp(pSetpoint, Wrist.kReverseSoftLimit, Wrist.kForwardSoftLimit);
    this.mProfiledPIDController =
        new ProfiledPIDController(
            Wrist.kP,
            0.0,
            Wrist.kD,
            new TrapezoidProfile.Constraints(Wrist.kMaxVelocity, Wrist.kMaxAcceleration));
    this.mProfiledPIDController.setTolerance(Wrist.kTolerance);

    addRequirements(pClawSubsystem);
  }

  @Override
  public void initialize() {
    mProfiledPIDController.reset(getMeasurement());
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    double calculatedOutput = mProfiledPIDController.calculate(getMeasurement(), mSetpoint);
    mClawSubsystem.setWrist(calculatedOutput);
  }

  @Override
  public void end(boolean interrupted) {
    mClawSubsystem.setWrist(0);
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return mProfiledPIDController.atSetpoint();
  }

  private double getMeasurement() {
    return mClawSubsystem.getEncoderMeasurement();
  }
}
