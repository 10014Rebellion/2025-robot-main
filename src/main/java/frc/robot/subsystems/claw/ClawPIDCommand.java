// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Wrist;

public class ClawPIDCommand extends Command {
  private final Claw mClawSubsystem;
  private double mSetpoint;
  private final PIDController mProfiledPIDController;
  private final ArmFeedforward mClawFeedforward;
  private final boolean IS_TUNING = false;

  public ClawPIDCommand(double pSetpoint, Claw pClawSubsystem) {
    this.mClawSubsystem = pClawSubsystem;

    if (IS_TUNING) {
      this.mSetpoint = SmartDashboard.getNumber("TunableNumbers/Tuning/Wrist/Setpoint", 0);
    } else {
      this.mSetpoint = MathUtil.clamp(pSetpoint, Wrist.kReverseSoftLimit, Wrist.kForwardSoftLimit);
    }

    this.mProfiledPIDController = new PIDController(Wrist.kP, 0.0, Wrist.kD);
    // new ProfiledPIDController(
    //     Wrist.kP,
    //     0.0,
    //     Wrist.kD,
    //     new TrapezoidProfile.Constraints(Wrist.kMaxVelocity, Wrist.kMaxAcceleration));
    this.mProfiledPIDController.setTolerance(Wrist.kTolerance);
    this.mClawFeedforward = new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);

    SmartDashboard.putNumber("Tuning/Wrist/Output Value", 0.0);

    addRequirements(pClawSubsystem);
  }

  @Override
  public void initialize() {
    mProfiledPIDController.setPID(Wrist.kP, 0, Wrist.kD);
    // mProfiledPIDController.setConstraints(
    //     new TrapezoidProfile.Constraints(Wrist.kMaxVelocity, Wrist.kMaxAcceleration));
    // mProfiledPIDController.reset(getMeasurement());
    mProfiledPIDController.reset();
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName()));
  }

  @Override
  public void execute() {

    if (IS_TUNING) {
      mSetpoint = SmartDashboard.getNumber("TunableNumbers/Tuning/Setpoint", 0);
    }

    double calculatedFeedforward = mClawFeedforward.calculate(getMeasurement(), 0.0);
    double calculatedProfilePID = mProfiledPIDController.calculate(getMeasurement(), mSetpoint);
    double calculatedOutput = calculatedProfilePID + calculatedFeedforward;
    mClawSubsystem.setWrist(calculatedOutput);
    SmartDashboard.putNumber("Tuning/Wrist/Output Value", calculatedOutput);
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
    return false; // mProfiledPIDController.atSetpoint();
  }

  private double getMeasurement() {
    return mClawSubsystem.getEncoderMeasurement();
  }
}
