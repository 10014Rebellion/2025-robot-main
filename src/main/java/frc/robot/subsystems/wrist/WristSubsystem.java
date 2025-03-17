// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  private final SparkMax mWristSparkMax;
  private final ProfiledPIDController mWristProfiledPID;
  private final ArmFeedforward mWristFF;
  private final DutyCycleEncoder mWristEncoder;
  private Controllers mCurrentController;

  private enum Controllers {
    ProfiledPID,
    Feedforward,
    Manual
  }

  public WristSubsystem() {
    this.mWristSparkMax = new SparkMax(WristConstants.kMotorID, WristConstants.kMotorType);
    this.mWristEncoder = new DutyCycleEncoder(WristConstants.kEncoderPort);
    this.mWristProfiledPID =
        new ProfiledPIDController(
            WristConstants.kP,
            0,
            WristConstants.kD,
            new Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration));
    this.mWristProfiledPID.setTolerance(WristConstants.kTolerance);
    this.mWristFF =
        new ArmFeedforward(
            WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);
    this.mCurrentController = Controllers.Manual;

    mWristEncoder.setDutyCycleRange(0, 1);
    mWristEncoder.setInverted(WristConstants.kEncoderInverted);

    this.mWristSparkMax.configure(
        WristConstants.kWristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    this.setDefaultCommand(enableFFCmd());
  }

  public FunctionalCommand enableFFCmd() {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.Feedforward;
        },
        () -> {
          double calculatedOutput = mWristFF.calculate(getEncReading(), 0);
          setVolts(calculatedOutput);
        },
        (interrupted) -> setVolts(0),
        () -> false,
        this);
  }

  public boolean isPIDAtGoal() {
    return mCurrentController.equals(Controllers.ProfiledPID) && mWristProfiledPID.atGoal();
  }

  public FunctionalCommand setPIDCmd(WristConstants.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.ProfiledPID;
          mWristProfiledPID.reset(getEncReading());
          mWristProfiledPID.setGoal(pSetpoint.getPos());
          SmartDashboard.putNumber("Wrist/Setpoint", pSetpoint.getPos());
        },
        () -> {
          double encoderReading = getEncReading();
          double calculatedFF =
              mWristFF.calculate(
                  Math.toRadians(mWristProfiledPID.getSetpoint().position),
                  Math.toRadians(mWristProfiledPID.getSetpoint().velocity));
          double calculatedPID = mWristProfiledPID.calculate(getEncReading());
          SmartDashboard.putNumber("Wrist/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Wrist/PID Output", calculatedPID);
          SmartDashboard.putNumber("Wrist/FF Output", calculatedFF);
          setVolts(calculatedPID + calculatedFF);
        },
        (interrupted) -> setVolts(0),
        () -> isPIDAtGoal(),
        this);
  }

  public FunctionalCommand setVoltsCmd(double pVoltage) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setVolts(pVoltage);
        },
        (interrupted) -> setVolts(0),
        () -> false,
        this);
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
    double encoderMeasurement =
        (getRawEncReading() * WristConstants.kPositionConversionFactor)
            + WristConstants.kEncoderOffsetDeg;
    if (encoderMeasurement > WristConstants.kPositionConversionFactor / 2.0)
      encoderMeasurement -= WristConstants.kPositionConversionFactor;
    return encoderMeasurement;
  }

  public double getRawEncReading() {
    return mWristEncoder.get();
  }

  @Override
  public void periodic() {
    stopIfLimit();
    SmartDashboard.putNumber("Wrist/Encoder", getEncReading());
  }
}
