// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax mElevatorSparkMax;
  private final SparkClosedLoopController mElevatorController;
  private final RelativeEncoder mEncoder;

  public Elevator() {
    mElevatorSparkMax = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    mEncoder = mElevatorSparkMax.getEncoder();
    mElevatorController = mElevatorSparkMax.getClosedLoopController();

    mElevatorSparkMax.configure(
        ElevatorConstants.kElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    SmartDashboard.putNumber("Target Position", 0);
    SmartDashboard.putNumber("Target Velocity", 0);
  }

  public void setMotor(double pVoltage) {
    mElevatorSparkMax.setVoltage(-filterVoltage(pVoltage));
  }

  private double filterVoltage(double pVoltage) {
    return (MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  // private double filterToLimits(double pInput) {
  //   return (pInput > 0 && mEncoder.getPosition() >= ElevatorConstants.kForwardSoftLimit)
  //           || (pInput < 0 && mEncoder.getPosition() <= ElevatorConstants.kReverseSoftLimit)
  //       ? 0.0
  //       : pInput;
  // }

  public void goToSetpoint(double pSetpoint) {
    mElevatorController.setReference(pSetpoint, ControlType.kMAXMotionPositionControl);
  }

  public boolean isAtSetpoint(double pSetpoint) {
    return (mEncoder.getPosition() >= pSetpoint - ElevatorConstants.kTolerance)
        && (mEncoder.getPosition() <= pSetpoint + ElevatorConstants.kTolerance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Actual Position", mEncoder.getPosition());
    SmartDashboard.putNumber("Actual Velocity", mEncoder.getVelocity());
  }
}
