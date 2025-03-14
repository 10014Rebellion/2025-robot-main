// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class WristSubsystem extends SubsystemBase {
  private final SparkMax mWristSparkMax;
  private final ProfiledPIDController mWristProfiledPID;
  private final ArmFeedforward mWristFF;
  private final SparkAbsoluteEncoder mWristEncoder;
  private double mWristSetpoint;
  private Controllers mCurrentController;

  private enum Controllers {
    ProfiledPID,
    Feedforward,
    Manual
  }

  public WristSubsystem() {
    this.mWristSparkMax = new SparkMax(WristConstants.kMotorID, WristConstants.kMotorType);
    this.mWristEncoder = mWristSparkMax.getAbsoluteEncoder();
    this.mWristProfiledPID = new ProfiledPIDController(WristConstants.kP, 0, WristConstants.kD, new Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration));
    this.mWristFF = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);
    this.mCurrentController = Controllers.Manual;
    this.mWristSetpoint = 0;

    mWristSparkMax.configure(
        WristConstants.kWristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setTriggers() {
    new Trigger(() -> isPIDAtGoal()).onTrue(null);
  }

  // TODO: IMPLEMENT THE PIVOT OFFSET ON HERE
  public FunctionalCommand enableFFCmd() {
    return new FunctionalCommand(() -> {
      mCurrentController = Controllers.Feedforward;
    }, () -> {
      double calculatedOutput = mWristFF.calculate(getEncReading(), 0);
      setVolts(calculatedOutput);
    }, (interrupted) -> {
      setVolts(0);
    }, () -> isPIDAtGoal(), this);
  }

  public boolean isPIDAtGoal() {
    return mWristProfiledPID.atGoal();
  }

  // TODO: IMPLEMENT THE PIVOT OFFSET ON HERE
  public FunctionalCommand setPIDCmd(WristConstants.Setpoints setpoint) {
    return new FunctionalCommand(() -> {
      mCurrentController = Controllers.ProfiledPID;
      mWristProfiledPID.setGoal(setpoint.getPos());
    }, () -> {
      double encoderReading = getEncReading();
      double calculatedPID = mWristFF.calculate(encoderReading, 0.0);
      double calculatedFF = mWristProfiledPID.calculate(encoderReading);
      double calculatedOutput = calculatedPID + calculatedFF;

      setVolts(calculatedOutput);
    }, (interrupted) -> {
      setVolts(0);
    }, () -> isPIDAtGoal(), this);
  }

  public void setVoltsCmd(double pVoltage) {
    new FunctionalCommand(() -> {
      mCurrentController = Controllers.Manual;
    }, () -> {
      setVolts(pVoltage);
    }, (interrupted) -> {}, () -> false, this);
  }

  public void setVolts(double pVoltage) {
    mWristSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  public double getMotorOutput() {
    return mWristSparkMax.getAppliedOutput();
  }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  private boolean isOutOfBounds(double pInput) {
    return (pInput > 0 && getEncReading() >= WristConstants.kForwardSoftLimit)
        || (pInput < 0 && getEncReading() <= WristConstants.kReverseSoftLimit);
  }

  private double filterToLimits(double pInput) {
    return isOutOfBounds(pInput) ? 0.0 : pInput;
  }

  private void stopIfLimit() {
    if (isOutOfBounds(getMotorOutput())) {
      setVolts(0);
    }
  }

  public double getEncReading() {
    double encoderMeasurement = mWristEncoder.getPosition();
    if (encoderMeasurement > WristConstants.kPositionConversionFactor / 2.0)
      encoderMeasurement -= WristConstants.kPositionConversionFactor;
    return encoderMeasurement;
  }

  public double getRawEncReading() {
    double encoderMeasurement = mWristEncoder.getPosition();
    if (encoderMeasurement > WristConstants.kPositionConversionFactor / 2.0)
      encoderMeasurement -= WristConstants.kPositionConversionFactor;
    return encoderMeasurement;
  }

  @Override
  public void periodic() {
    stopIfLimit();
  }
}
