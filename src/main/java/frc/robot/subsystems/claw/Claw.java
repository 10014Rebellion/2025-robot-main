// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawConstants.Wrist;

public class Claw extends SubsystemBase {
  private final SparkMax mWristSparkMax;
  private final SparkFlex mLeftClawSparkMax;
  private final SparkFlex mRightClawSparkMax;

  private final SparkClosedLoopController mWristController;
  private final AbsoluteEncoder mWristEncoder;

  public Claw() {
    this.mLeftClawSparkMax =
        new SparkFlex(ClawConstants.Claw.kLeftClawID, ClawConstants.Claw.kMotorType);
    this.mRightClawSparkMax =
        new SparkFlex(ClawConstants.Claw.kRightClawID, ClawConstants.Claw.kMotorType);

    this.mWristSparkMax = new SparkMax(Wrist.kMotorID, Wrist.kMotorType);
    this.mWristController = mWristSparkMax.getClosedLoopController();
    this.mWristEncoder = mWristSparkMax.getAbsoluteEncoder();

    mWristSparkMax.configure(
        Wrist.kWristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    mLeftClawSparkMax.configure(
        ClawConstants.Claw.kClawConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    mRightClawSparkMax.configure(
        ClawConstants.Claw.kClawConfig.inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void setClaw(double pVoltage) {
    mLeftClawSparkMax.setVoltage(filterVoltage(pVoltage));
    mRightClawSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  public void setMotor(double pVoltage) {
    mWristSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  private double filterVoltage(double pVoltage) {
    return (MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getEncoderMeasurement() {
    return mWristEncoder.getPosition();
  }

  // private double filterToLimits(double pInput) {
  //   return (pInput > 0 && mWristEncoder.getPosition() >= ElevatorConstants.kForwardSoftLimit)
  //           || (pInput < 0 && mWristEncoder.getPosition() <= ElevatorConstants.kReverseSoftLimit)
  //       ? 0.0
  //       : pInput;
  // }

  public void goToSetpoint(double pSetpoint) {
    mWristController.setReference(pSetpoint, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", mWristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Voltage", mWristSparkMax.getBusVoltage());
  }
}
