// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLogOutput;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.controls.StateTracker.CoralLevel;
import frc.robot.util.debugging.LoggedTunableNumber;

public class WristSubsystem extends SubsystemBase {
  @AutoLogOutput(key = "Wrist/Slot")
  public ClosedLoopSlot mCurrentSlot = ClosedLoopSlot.kSlot0;
  private WristConstants.Setpoints mCurrentSetpoint = null;
  private static final LoggedTunableNumber k2P = new LoggedTunableNumber("Wrist/Slot2/P", WristConstants.k2P);
  private static final LoggedTunableNumber k2I = new LoggedTunableNumber("Wrist/Slot2/I", WristConstants.k2I);
  private static final LoggedTunableNumber k2D = new LoggedTunableNumber("Wrist/Slot2/D", WristConstants.k2D);
  private static final LoggedTunableNumber k2MaxV = new LoggedTunableNumber("Wrist/Slot2/MaxV", WristConstants.k2MaxVelocity);
  private static final LoggedTunableNumber k2MaxA = new LoggedTunableNumber("Wrist/Slot2/MaxA", WristConstants.k2MaxAcceleration);
  private static final LoggedTunableNumber k2S = new LoggedTunableNumber("Wrist/Slot2/S", WristConstants.k2S);
  private static final LoggedTunableNumber k2V = new LoggedTunableNumber("Wrist/Slot2/V", WristConstants.k2V);
  private static final LoggedTunableNumber k2A = new LoggedTunableNumber("Wrist/Slot2/A", WristConstants.k2A);
  private static final LoggedTunableNumber k2G = new LoggedTunableNumber("Wrist/Slot2/G", WristConstants.k2G);

  private final SparkMax mWristSparkMax;
  private final DutyCycleEncoder mWristEncoder;
  private final SparkClosedLoopController mWristSparkController;


  public WristSubsystem() {
    this.mWristSparkMax = new SparkMax(WristConstants.kMotorID, WristConstants.kMotorType);
    this.mWristEncoder = new DutyCycleEncoder(WristConstants.kEncoderPort);
    this.mWristEncoder.setDutyCycleRange(0, 1);
    this.mWristEncoder.setInverted(WristConstants.kEncoderInverted);
  
    this.mWristSparkMax.configure(
        WristConstants.kWristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.mWristSparkController = mWristSparkMax.getClosedLoopController();
    this.setDefaultCommand(defaultPIDCommand());
  }

  public InstantCommand setPIDSetpoint(WristConstants.Setpoints pWristSetpoint) {
    return new InstantCommand(() -> mCurrentSetpoint = pWristSetpoint);
  }

  private FunctionalCommand defaultPIDCommand() {
    return new FunctionalCommand(
        () -> {
          System.out.println("Wrist PID Running");
        },
        () -> {
          if(mCurrentSetpoint == null)
            mWristSparkMax.stopMotor();
          else 
            mWristSparkController.setSetpoint(mCurrentSetpoint.getPos(), ControlType.kMAXMotionPositionControl, mCurrentSlot);
        },
        (interrupted) -> {},
        () -> false,
        this);
  }

  public boolean isPIDAtGoal() {
    return mWristSparkController.isAtSetpoint();
  }

  public Command coralLevelToPIDCmd(CoralLevel pCoralLevel) {
    WristConstants.Setpoints wristSetpoint = WristConstants.Setpoints.L1;

    if(pCoralLevel == CoralLevel.B3) {
      wristSetpoint = WristConstants.Setpoints.L4;
    } else
    if(pCoralLevel == CoralLevel.B2) {
      wristSetpoint = WristConstants.Setpoints.L3;
    } else
    if(pCoralLevel == CoralLevel.B1) {
      wristSetpoint = WristConstants.Setpoints.L2;
    } 

    return setPIDSetpoint(wristSetpoint);
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
    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      setTunablePID(k2P.get(), k2I.get(), k2D.get(), k2MaxV.get(), k2MaxA.get(), k2S.get(), k2V.get(), k2A.get(), k2G.get());
    }, k2P, k2I, k2D, k2MaxV, k2MaxA, k2S, k2V, k2A, k2G);

    SmartDashboard.putNumber("Wrist/Encoder", getEncReading());
    SmartDashboard.putNumber("Wrist/Position Setpoint", mWristSparkController.getMAXMotionSetpointPosition());
    SmartDashboard.putNumber("Wrist/Velocity Setpoint", mWristSparkController.getMAXMotionSetpointVelocity());    
    SmartDashboard.putBoolean("Wrist/At Setpoint", isPIDAtGoal());

    stopIfLimit();
  }

  private void setTunablePID(double kP, double kI, double kD, double kMaxV, double kMaxA, double kS, double kV, double kA, double kG) {
    WristConstants.kWristConfig.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot2);
    WristConstants.kWristConfig.closedLoop.maxMotion.cruiseVelocity(kMaxV, ClosedLoopSlot.kSlot2);
    WristConstants.kWristConfig.closedLoop.maxMotion.maxAcceleration(kMaxA, ClosedLoopSlot.kSlot2);
    WristConstants.kWristConfig.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot2).kV(kV, ClosedLoopSlot.kSlot2).kA(kA, ClosedLoopSlot.kSlot2).kG(kG, ClosedLoopSlot.kSlot2);
    mWristSparkMax.configure(WristConstants.kWristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
