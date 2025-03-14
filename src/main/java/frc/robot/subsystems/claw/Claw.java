// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;
import frc.robot.subsystems.claw.ClawConstants.Wrist;
import frc.robot.util.TunableNumber;

public class Claw extends SubsystemBase {
  private final SparkFlex mWristSparkMax;
  private final SparkFlex mLeftClawSparkMax;

  private final SparkClosedLoopController mWristController;
  // private AbsoluteEncoder mWristEncoder;

  private final DutyCycleEncoder mWristEncoder;

  private TunableNumber tunablePosition;

  private AnalogInput mUltrasonic;

  private double previousPosition, previousTime, previousVelocity;
  private double velocity, acceleration;

  public Claw() {
    this.mLeftClawSparkMax = new SparkFlex(70, ClawConstants.Claw.kMotorType);
    this.mWristEncoder = new DutyCycleEncoder(5);

    this.mWristSparkMax = new SparkFlex(71, Wrist.kMotorType);
    this.mWristController = mWristSparkMax.getClosedLoopController();
    // this.mWristEncoder = mWristSparkMax.getAbsoluteEncoder();

    mWristSparkMax.configure(
        Wrist.kWristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    mLeftClawSparkMax.configure(
        ClawConstants.Claw.kClawConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    mUltrasonic = new AnalogInput(0);

    SmartDashboard.putNumber("Wrist/Tuning/kP", ClawConstants.Wrist.kP);
    SmartDashboard.putNumber("Wrist/Tuning/kD", ClawConstants.Wrist.kD);
    SmartDashboard.putNumber("Wrist/Tuning/kG", ClawConstants.Wrist.kG);
    SmartDashboard.putNumber("Wrist/Tuning/kV", ClawConstants.Wrist.kV);
    SmartDashboard.putNumber("Wrist/Tuning/kA", ClawConstants.Wrist.kA);
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
    mLeftClawSparkMax.setVoltage(pVoltage);
  }

  public void setWrist(double pVoltage) {
    mWristSparkMax.setVoltage(pVoltage);
  }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getEncoderMeasurement() {
    double encoderMeasurement = -(mWristEncoder.get() * 360) + ClawConstants.Wrist.kEncoderOffset;
    if (encoderMeasurement > 180) encoderMeasurement -= 360;
    return encoderMeasurement;
  }

  public void updateVelocity() {
    double currentTime = Timer.getFPGATimestamp();
    double currentPosition = getEncoderMeasurement();

    // Calculate velocity: ΔPosition / ΔTime
    double deltaPosition = currentPosition - previousPosition;
    double deltaTime = currentTime - previousTime;

    if (deltaTime > 0) {
      velocity = deltaPosition / deltaTime; // Velocity in terms of revolutions per second (rps)
    }

    // Calculate acceleration: ΔVelocity / ΔTime
    double deltaVelocity = velocity - previousVelocity;
    if (deltaTime > 0) {
      acceleration = deltaVelocity / deltaTime; // Acceleration in terms of rps^2
    }

    // Update previous values for the next cycle
    previousPosition = currentPosition;
    previousTime = currentTime;
    previousVelocity = velocity;
  }

  public double getVelocity() {
    return velocity;
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

  @Override
  public void periodic() {
    stopIfLimit();
    updateVelocity();
    // SmartDashboard.putNumber("Wrist/Position", getEncoderMeasurement());
    // SmartDashboard.putNumber("Wrist/Velocity", getVelocity());
    // SmartDashboard.putNumber("Wrist/Voltage", mWristSparkMax.getBusVoltage());
    // SmartDashboard.putNumber("Wrist/Ultrasonic", getUltrasonicDistance());
  }
}
