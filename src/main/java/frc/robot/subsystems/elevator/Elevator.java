// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax mElevatorSparkMax;
  private final SparkClosedLoopController mElevatorController;
  private final RelativeEncoder mEncoder;

  private TunableNumber elevatorFF, elevatorP, elevatorI, elevatorD;
  private TunableNumber elevatorVelocity, elevatorAcceleration;

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

    elevatorVelocity = new TunableNumber("Elevator/Elevator Velocity");
    elevatorAcceleration = new TunableNumber("Elevator/Elevator Acceleration");

    elevatorFF.setDefault(ElevatorConstants.kG);
    elevatorP.setDefault(ElevatorConstants.kP);
    elevatorI.setDefault(0);
    elevatorD.setDefault(ElevatorConstants.kD);

    elevatorVelocity.setDefault(ElevatorConstants.kMaxVelocity);
    elevatorAcceleration.setDefault(ElevatorConstants.kMaxAcceleration);
  }

  public void setMotorVoltage(double pVoltage) {
    // if (pVoltage < 0.0) pVoltage *= 0.2; // Slows down downward movements
    mElevatorSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  public Command stopPID() {
    return this.runOnce(
        () -> {
          mElevatorController.setReference(0, ControlType.kMAXMotionVelocityControl);
        });
  }

  public Command goTo(double setpoint) {
    return this.run(
        () -> {
          mElevatorController.setReference(
              setpoint,
              ControlType.kMAXMotionPositionControl,
              ClosedLoopSlot.kSlot0,
              ElevatorConstants.kG,
              ArbFFUnits.kVoltage);
        });
  }

  public Command holdElevator() {
    return this.run(
        () -> {
          mElevatorController.setReference(
              getEncoderMeasurement(),
              ControlType.kMAXMotionPositionControl,
              ClosedLoopSlot.kSlot0,
              ElevatorConstants.kG,
              ArbFFUnits.kVoltage);
        });
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

  public boolean isAtSetpoint(double pSetpoint) {
    return (mEncoder.getPosition() >= pSetpoint - ElevatorConstants.kTolerance)
        && (mEncoder.getPosition() <= pSetpoint + ElevatorConstants.kTolerance);
  }

  public double getMotorOutput() {
    return mElevatorSparkMax.getAppliedOutput();
  }

  // These just act as ways to tune the PID quickly and easily.
  public void setElevatorP() {
    ElevatorConstants.kP = elevatorP.get();
  }

  public void setElevatorD() {
    ElevatorConstants.kD = elevatorD.get();
  }

  public void setElevatorVelocity() {
    ElevatorConstants.kMaxVelocity = elevatorVelocity.get();
  }

  public void setElevatorAcceleration() {
    ElevatorConstants.kMaxAcceleration = elevatorAcceleration.get();
  }

  @Override
  public void periodic() {
    stopIfLimit();

    SmartDashboard.putNumber("Elevator Position", mEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", mEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Output", getMotorOutput());
    SmartDashboard.putNumber("Elevator Voltage", mElevatorSparkMax.getBusVoltage());

    if(elevatorP.hasChanged()) setElevatorP();
    if(elevatorD.hasChanged()) setElevatorD();
    if(elevatorVelocity.hasChanged()) setElevatorVelocity();
    if(elevatorAcceleration.hasChanged()) setElevatorAcceleration();
  }
}
