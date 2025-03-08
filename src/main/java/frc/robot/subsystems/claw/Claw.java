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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;
import frc.robot.subsystems.claw.ClawConstants.Wrist;
import frc.robot.util.TunableNumber;

public class Claw extends SubsystemBase {
  private final SparkMax mWristSparkMax;
  private final SparkFlex mLeftClawSparkMax;
  private final SparkFlex mRightClawSparkMax;

  private final SparkClosedLoopController mWristController;
  private AbsoluteEncoder mWristEncoder;

  private final DutyCycleEncoder mClawEncoder;

  private TunableNumber wristP, wristD, wristG, wristV, wristA;
  private TunableNumber tunablePosition;

  private AnalogInput mUltrasonic;

  public Claw() {
    this.mLeftClawSparkMax =
        new SparkFlex(ClawConstants.Claw.kLeftClawID, ClawConstants.Claw.kMotorType);
    this.mRightClawSparkMax =
        new SparkFlex(ClawConstants.Claw.kRightClawID, ClawConstants.Claw.kMotorType);
    this.mClawEncoder = new DutyCycleEncoder(ClawConstants.Claw.kEncoderDIOPort);

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
    mUltrasonic = new AnalogInput(0);

    wristP = new TunableNumber("Wrist/kP", Wrist.kP);
    wristD = new TunableNumber("Wrist/kD", Wrist.kD);
    wristV = new TunableNumber("Wrist/kVelocity", Wrist.kV);
    wristA = new TunableNumber("Wrist/kAcceleration", Wrist.kA);
    // tunablePosition = new TunableNumber("Wrist/Tunable Setpoint", 0);
    // wristP.setDefault(0.0);
    // wristD.setDefault(0.0);
    // wristV.setDefault(0.0);
    // wristA.setDefault(0.0);
  }

  public void setClaw(ClawRollerVolt pVoltage) {
    setClaw(pVoltage.get());
  }

  public void setClaw(double pVoltage) {
    mLeftClawSparkMax.setVoltage(filterVoltage(pVoltage));
    mRightClawSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  public double getClaw() {
    double measurement = (mClawEncoder.get() * 360.0) + ClawConstants.Claw.kEncoderOffset;
    if (measurement >= 180) {
      return measurement - 360;
    }
    return measurement;
  }

  public void setWrist(double pVoltage) {
    mWristSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getEncoderMeasurement() {
    double encoderMeasurement = mWristEncoder.getPosition();
    if (encoderMeasurement > ClawConstants.Wrist.kPositionConversionFactor / 2.0)
      encoderMeasurement -= ClawConstants.Wrist.kPositionConversionFactor;
    return encoderMeasurement;
  }

  private double filterToLimits(double pInput) {
    return (pInput > 0 && getEncoderMeasurement() >= ClawConstants.Wrist.kForwardSoftLimit)
            || (pInput < 0 && getEncoderMeasurement() <= ClawConstants.Wrist.kReverseSoftLimit)
        ? 0.0
        : pInput;
  }

  private void stopIfLimit() {
    double motorOutput = getMotorOutput();
    if ((motorOutput > 0 && getEncoderMeasurement() >= ClawConstants.Wrist.kForwardSoftLimit)
        || (motorOutput < 0 && getEncoderMeasurement() <= ClawConstants.Wrist.kReverseSoftLimit)) {
      setWrist(0);
    }
  }

  public double getMotorOutput() {
    return mWristSparkMax.getAppliedOutput();
  }

  public void goToSetpoint(double pSetpoint) {
    mWristController.setReference(pSetpoint, ControlType.kMAXMotionPositionControl);
  }

  public double getUltrasonicDistance() {
    double voltage_scale_factor = 5 / RobotController.getVoltage5V();
    double currentDistanceCM = mUltrasonic.getValue() * voltage_scale_factor * 0.125;
    return currentDistanceCM;
  }

  public boolean hasCoral() {
    return Math.abs(getClaw() - ClawConstants.Claw.ClawOpenPositions.HAS_CORAL.get())
        < ClawConstants.Claw.positionTolerance;
  }

  public boolean isClawOpen() {
    return getClaw() > ClawConstants.Claw.ClawOpenPositions.OPEN.get();
  }

  @Override
  public void periodic() {
    stopIfLimit();
    SmartDashboard.putNumber("Wrist/Position", getEncoderMeasurement());
    SmartDashboard.putNumber("Wrist/Voltage", mWristSparkMax.getBusVoltage());
    SmartDashboard.putNumber("Wrist/Ultrasonic", getUltrasonicDistance());
    SmartDashboard.putNumber("Claw/Open Value", getClaw());
    SmartDashboard.putBoolean("Claw/Periodic Has Coral", hasCoral());
    ClawConstants.Claw.periodicHasCoral = hasCoral();

    if (wristP.hasChanged()) Wrist.kP = wristP.get();
    // SmartDashboard.putNumber("Tuning/Wrist/Current P", Wrist.kP);
    if (wristD.hasChanged()) Wrist.kD = wristD.get();
    if (wristV.hasChanged()) Wrist.kMaxVelocity = wristV.get();
    if (wristD.hasChanged()) Wrist.kMaxAcceleration = wristA.get();
  }
}
