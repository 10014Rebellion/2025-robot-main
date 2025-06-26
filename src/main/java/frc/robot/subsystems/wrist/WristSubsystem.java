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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.controls.StateTracker.CoralLevel;
public class WristSubsystem extends SubsystemBase {
  private final SparkMax mWristSparkMax;
  private final ProfiledPIDController mWristProfiledPID;
  private ArmFeedforward mWristFF;
  private final DutyCycleEncoder mWristEncoder;

  public WristSubsystem() {
    this.mWristSparkMax = new SparkMax(WristConstants.kMotorID, WristConstants.kMotorType);
    this.mWristEncoder = new DutyCycleEncoder(WristConstants.kEncoderPort);
    this.mWristProfiledPID = new ProfiledPIDController(
        WristConstants.kP,
        0,
        WristConstants.kD,
        new Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration));
    this.mWristProfiledPID.setTolerance(WristConstants.kTolerance);
    this.mWristFF = new ArmFeedforward(
        WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);

    mWristEncoder.setDutyCycleRange(0, 1);
    mWristEncoder.setInverted(WristConstants.kEncoderInverted);

    this.mWristSparkMax.configure(
        WristConstants.kWristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    this.setDefaultCommand(enableFFCmd());

    SmartDashboard.putNumber("Wrist/kP", WristConstants.kP);
    SmartDashboard.putNumber("Wrist/kD", WristConstants.kD);
    SmartDashboard.putNumber("Wrist/kG", WristConstants.kG);
    SmartDashboard.putNumber("Wrist/kV", WristConstants.kV);
    SmartDashboard.putNumber("Wrist/kA", WristConstants.kA);
    SmartDashboard.putNumber("Wrist/kS", WristConstants.kS);

    SmartDashboard.putNumber("Wrist/Tunable Setpoint", 0.0);
    SmartDashboard.putNumber("Wrist/Max Vel", WristConstants.kMaxVelocity);
    SmartDashboard.putNumber("Wrist/Max Accel", WristConstants.kMaxAcceleration);
  }

  public FunctionalCommand enableFFCmd() {

    return new FunctionalCommand(
        () -> {
          System.out.println("Wrist FF Running");
        },
        () -> {
          double calculatedOutput = mWristFF.calculate(Units.degreesToRadians(getEncReading()), 0);
          setVolts(calculatedOutput);
        },
        (interrupted) -> setVolts(mWristFF.calculate(Units.degreesToRadians(getEncReading()), 0)),
        () -> false,
        this);
  }

  public boolean isPIDAtGoal() {
    return mWristProfiledPID.atGoal();
  }

  public FunctionalCommand setTunablePIDCmd(double pSetpoint) {
    return new FunctionalCommand(
        () -> {
          double newKp = SmartDashboard.getNumber("Wrist/kP", WristConstants.kP);
          double newKd = SmartDashboard.getNumber("Wrist/kD", WristConstants.kD);
          // double pSetpoint = SmartDashboard.getNumber("Wrist/Tunable Setpoint", 0.0);

          double newVel = SmartDashboard.getNumber("Wrist/Max Vel", WristConstants.kMaxVelocity);
          double newAccel = SmartDashboard.getNumber("Wrist/Max Accel", WristConstants.kMaxAcceleration);
          double newkG = SmartDashboard.getNumber("Wrist/kG", WristConstants.kG);
          double newkV = SmartDashboard.getNumber("Wrist/kV", WristConstants.kV);
          double newkA = SmartDashboard.getNumber("Wrist/kA", WristConstants.kA);
          double newkS = SmartDashboard.getNumber("Wrist/kS", WristConstants.kS);

          mWristProfiledPID.setConstraints(new Constraints(newVel, newAccel));
          mWristProfiledPID.setPID(newKp, 0.0, newKd);
          mWristProfiledPID.reset(getEncReading());
          mWristProfiledPID.setGoal(pSetpoint);

          mWristFF = new ArmFeedforward(newkS, newkG, newkV, newkA);
        },
        () -> {
          double encoderReading = getEncReading();
          double calculatedPID = mWristProfiledPID.calculate(encoderReading);
          double calculatedFF = mWristFF.calculate(
              Units.degreesToRadians(mWristProfiledPID.getSetpoint().position),
              Units.degreesToRadians(mWristProfiledPID.getSetpoint().velocity));

          setVolts(calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Wrist/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Wrist/PID Output", calculatedPID);
          SmartDashboard.putNumber("Wrist/FF Output", calculatedFF);
        },
        (interrupted) -> setVolts(0),
        () -> isPIDAtGoal(),
        this);
  }

  public FunctionalCommand coralLevelToPIDCmd(CoralLevel pCoralLevel) {
    WristConstants.Setpoints elevatorSetpoint = WristConstants.Setpoints.L1;

    if(pCoralLevel == CoralLevel.B3) {
      elevatorSetpoint = WristConstants.Setpoints.L4;
    } else
    if(pCoralLevel == CoralLevel.B2) {
      elevatorSetpoint = WristConstants.Setpoints.L3;
    } else
    if(pCoralLevel == CoralLevel.B1) {
      elevatorSetpoint = WristConstants.Setpoints.L2;
    } 

    return setPIDCmd(elevatorSetpoint);
  }


  // public FunctionalCommand setTunableCommand

  public FunctionalCommand setPIDCmd(WristConstants.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          SmartDashboard.putNumber("Wrist/Setpoint", pSetpoint.getPos());
          mWristProfiledPID.reset(getEncReading());
          mWristProfiledPID.setGoal(pSetpoint.getPos());
        },
        () -> {
          double calculatedFF = mWristFF.calculate(
              Math.toRadians(mWristProfiledPID.getSetpoint().position),
              Math.toRadians(mWristProfiledPID.getSetpoint().velocity));
          double calculatedPID = mWristProfiledPID.calculate(getEncReading());
          SmartDashboard.putNumber("Wrist/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Wrist/PID Output", calculatedPID);
          SmartDashboard.putNumber("Wrist/FF Output", calculatedFF);
          setVolts(calculatedPID + calculatedFF);
        },
        (interrupted) -> setVolts(mWristFF.calculate(Math.toRadians(getEncReading()), 0.0)),
        () -> isPIDAtGoal(),
        this);
  }

  public FunctionalCommand setVoltsCmd(double pVoltage) {
    return new FunctionalCommand(
        () -> {
        },
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
    double encoderMeasurement = (getRawEncReading() * WristConstants.kPositionConversionFactor)
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
    SmartDashboard.putNumber("Wrist/Position Setpoint", mWristProfiledPID.getSetpoint().position);
    SmartDashboard.putNumber("Wrist/Velocity Setpoint", mWristProfiledPID.getSetpoint().velocity);

    SmartDashboard.putBoolean("Wrist/At Setpoint", isPIDAtGoal());
  }
}
