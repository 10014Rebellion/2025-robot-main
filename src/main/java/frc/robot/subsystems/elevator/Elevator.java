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
import frc.robot.util.TunableNumber;

public class Elevator extends SubsystemBase {
  private final SparkMax mElevatorSparkMax;
  private final SparkClosedLoopController mElevatorController;
  private final RelativeEncoder mEncoder;
  private double motorVoltage = 0;

  private TunableNumber elevatorFF, elevatorP, elevatorI, elevatorD;
  private TunableNumber elevatorTunableSetpoint;

  public Elevator() {
    mElevatorSparkMax = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    mEncoder = mElevatorSparkMax.getEncoder();
    mElevatorController = mElevatorSparkMax.getClosedLoopController();

    mElevatorSparkMax.configure(
        ElevatorConstants.kElevatorConfig.inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    SmartDashboard.putNumber("Target Position", 0);
    SmartDashboard.putNumber("Target Velocity", 0);

    elevatorFF = new TunableNumber("Elevator/Elevator FF");
    elevatorP = new TunableNumber("Elevator/Elevator P");
    elevatorI = new TunableNumber("Elevator/Elevator I");
    elevatorD = new TunableNumber("Elevator/Elevator D");
    elevatorTunableSetpoint = new TunableNumber("Elevator/Setpoint");

    elevatorFF.setDefault(ElevatorConstants.kG);
    elevatorP.setDefault(ElevatorConstants.kP);
    elevatorI.setDefault(0);
    elevatorD.setDefault(ElevatorConstants.kD);
    elevatorTunableSetpoint.setDefault(getEncoderMeasurement());
  }

  public void setMotorVoltage(double pVoltage) {
    // if (pVoltage < 0.0) pVoltage *= 0.2; // Slows down downward movements
    mElevatorSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  public void stepMotorVoltage(double value) {
    motorVoltage += value;
    setMotorVoltage(motorVoltage);
  }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getEncoderMeasurement() {
    return mEncoder.getPosition();
  }

  private double filterToLimits(double pInput) {
    return (pInput > 0 && mEncoder.getPosition() >= ElevatorConstants.kForwardSoftLimit)
            || (pInput < 0 && mEncoder.getPosition() <= ElevatorConstants.kReverseSoftLimit)
        ? 0.0
        : pInput;
  }

  private void stopIfLimit() {
    double motorOutput = getMotorOutput();
    if ((motorOutput > 0 && mEncoder.getPosition() >= ElevatorConstants.kForwardSoftLimit)
        || (motorOutput < 0 && mEncoder.getPosition() <= ElevatorConstants.kReverseSoftLimit)) {
      setMotorVoltage(0);
    }
  }

  public void goToSetpoint(double pSetpoint) {
    mElevatorController.setReference(pSetpoint, ControlType.kMAXMotionPositionControl);
  }

  public boolean isAtSetpoint(double pSetpoint) {
    return (mEncoder.getPosition() >= pSetpoint - ElevatorConstants.kTolerance)
        && (mEncoder.getPosition() <= pSetpoint + ElevatorConstants.kTolerance);
  }

  public double getMotorOutput() {
    return mElevatorSparkMax.getAppliedOutput();
  }

  @Override
  public void periodic() {
    stopIfLimit();

    SmartDashboard.putNumber("Elevator Position", mEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", mEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Output", getMotorOutput());
    SmartDashboard.putNumber("Elevator Voltage", mElevatorSparkMax.getBusVoltage());
    SmartDashboard.putNumber("Stepped Elevator Voltage", motorVoltage);
  }
}
