// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.potentiometer.Potentiometer;

public class ElevatorPID extends Command {
  private final Elevator mClawSubsystem;
  private final double mSetpoint;
  private final ProfiledPIDController mProfiledPIDController;

  public ElevatorPID(double pSetpoint, Elevator pClawSubsystem) {
    this.mClawSubsystem = pClawSubsystem;
    this.mSetpoint =
        MathUtil.clamp(
            pSetpoint, ElevatorConstants.kReverseSoftLimit, ElevatorConstants.kForwardSoftLimit);
    this.mProfiledPIDController =
        new ProfiledPIDController(
            ElevatorConstants.kP,
            0.0,
            ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
    this.mProfiledPIDController.setTolerance(ElevatorConstants.kTolerance);

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
    double potentiometerReading = Potentiometer.getPotentiometer();
    mProfiledPIDController.setP(potentiometerReading);

    double calculatedOutput = mProfiledPIDController.calculate(getMeasurement(), mSetpoint);
    mClawSubsystem.setMotorVoltage(calculatedOutput);

    SmartDashboard.putNumber("PID Output", calculatedOutput);
  }

  @Override
  public void end(boolean interrupted) {
    mClawSubsystem.setMotorVoltage(0);
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
