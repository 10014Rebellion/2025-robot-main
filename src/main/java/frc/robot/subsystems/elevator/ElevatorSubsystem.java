// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax mElevatorSparkMax;
  private final ProfiledPIDController mElevatorProfiledPID;
  private final ElevatorFeedforward mElevatorFF;

  private final RelativeEncoder mEncoder;

  private Controllers mCurrentController;

  private enum Controllers {
    ProfiledPID,
    Feedforward,
    Manual
  }

  public ElevatorSubsystem() {
    this.mElevatorSparkMax = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    this.mEncoder = mElevatorSparkMax.getEncoder();
    this.mElevatorProfiledPID =
        new ProfiledPIDController(
            ElevatorConstants.kP,
            0,
            ElevatorConstants.kD,
            new Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
    this.mElevatorFF =
        new ElevatorFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
    this.mCurrentController = Controllers.Manual;

    this.setDefaultCommand(enableFFCmd());

    this.mElevatorSparkMax.configure(
        ElevatorConstants.kElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public FunctionalCommand enableFFCmd() {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.Feedforward;
        },
        () -> {
          double calculatedOutput = mElevatorFF.calculate(0);
          setVolts(calculatedOutput);
        },
        (interrupted) -> setVolts(0),
        () -> false,
        this);
  }

  public boolean isPIDAtGoal() {
    return mCurrentController.equals(Controllers.ProfiledPID) && mElevatorProfiledPID.atGoal();
  }

  public FunctionalCommand setPIDCmd(ElevatorConstants.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.ProfiledPID;
          mElevatorProfiledPID.setGoal(pSetpoint.getPos());
        },
        () -> {
          double encoderReading = getEncReading();
          double calculatedPID = mElevatorFF.calculate(encoderReading, 0.0);
          double calculatedFF = mElevatorProfiledPID.calculate(encoderReading);
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
    mElevatorSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getRawEncReading() {
    return mEncoder.getPosition();
  }

  public double getEncReading() {
    return getRawEncReading() * ElevatorConstants.kPositionConversionFactor;
  }

  private boolean isOutOfBounds(double pInput) {
    return (pInput > 0 && getEncReading() >= ElevatorConstants.kForwardSoftLimit)
        || (pInput < 0 && getEncReading() <= ElevatorConstants.kReverseSoftLimit);
  }

  private double filterToLimits(double pInput) {
    return isOutOfBounds(pInput) ? 0.0 : pInput;
  }

  private void stopIfLimit() {
    if (isOutOfBounds(getMotorOutput())) {
      setVolts(0);
    }
  }

  public double getMotorOutput() {
    return mElevatorSparkMax.getAppliedOutput();
  }

  @Override
  public void periodic() {
    stopIfLimit();

    SmartDashboard.putNumber("Elevator/Position", getEncReading());
    // SmartDashboard.putNumber("Elevator/Velocity", mEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator/Output", getMotorOutput());
    SmartDashboard.putNumber("Elevator/Voltage", mElevatorSparkMax.getBusVoltage());
  }
}
