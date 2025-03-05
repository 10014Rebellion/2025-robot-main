// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants.Positions;
import java.util.function.Supplier;

public class NewElevatorPIDCommand extends Command {
  private final Elevator mElevatorSubsystem;
  private final Supplier<Positions> mSetpointSupplier;
  private final ProfiledPIDController mProfiledPIDController;
  private final ElevatorFeedforward mElevatorFeedforward;

  public NewElevatorPIDCommand(Supplier<Positions> pSetpointSupplier, Elevator pElevatorSubsystem) {
    this.mElevatorSubsystem = pElevatorSubsystem;
    this.mSetpointSupplier = pSetpointSupplier;
    this.mProfiledPIDController = new ProfiledPIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
    this.mElevatorFeedforward = new ElevatorFeedforward(
        ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
    this.mProfiledPIDController.setTolerance(ElevatorConstants.kTolerance);

    SmartDashboard.putNumber("Elevator/PID Output", 0.0);

    addRequirements(pElevatorSubsystem);
  }

  @Override
  public void initialize() {
    mProfiledPIDController.reset(getMeasurement());
    System.out.printf(
        "<<< %s - %s is STARTING :D >>>\n",
        this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName());
  }

  @Override
  public void execute() {
    double mSetpoint = mSetpointSupplier.get().getPos();

    double calculatedFeedforward = mElevatorFeedforward.calculate(0);
    double calculatedProfilePID = mProfiledPIDController.calculate(getMeasurement(), mSetpoint);
    double calculatedOutput = calculatedFeedforward + calculatedProfilePID;
    mElevatorSubsystem.setMotorVoltage(calculatedOutput);

    SmartDashboard.putNumber("Elevator/Calculated Output", calculatedOutput);
    SmartDashboard.putNumber("Elevator/Setpoint", mSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    mElevatorSubsystem.setMotorVoltage(mElevatorFeedforward.calculate(0.0));
    System.out.printf(
        "<<< %s - %s is ENDING :C >>>\n",
        this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName());
  }

  @Override
  public boolean isFinished() {
    return mProfiledPIDController.atGoal();
  }

  private double getMeasurement() {
    return mElevatorSubsystem.getEncoderMeasurement();
  }
}
