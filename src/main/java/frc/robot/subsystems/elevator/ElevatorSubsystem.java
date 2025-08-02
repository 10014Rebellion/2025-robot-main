// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.controls.StateTracker.CoralLevel;
import frc.robot.util.debugging.LoggedTunableNumber;

public class ElevatorSubsystem extends SubsystemBase {
  @AutoLogOutput(key = "Elevator/Slot")
  public int slot = 0;

  private static final LoggedTunableNumber k0P = new LoggedTunableNumber("Elevator/Slot0/P", ElevatorConstants.k0P);
  private static final LoggedTunableNumber k0I = new LoggedTunableNumber("Elevator/Slot0/I", ElevatorConstants.k0I);
  private static final LoggedTunableNumber k0D = new LoggedTunableNumber("Elevator/Slot0/D", ElevatorConstants.k0D);
  private static final LoggedTunableNumber k0MaxV = new LoggedTunableNumber("Elevator/Slot0/MaxV", ElevatorConstants.k0MaxVelocity);
  private static final LoggedTunableNumber k0MaxA = new LoggedTunableNumber("Elevator/Slot0/MaxA", ElevatorConstants.k0MaxAcceleration);
  private static final LoggedTunableNumber k0S = new LoggedTunableNumber("Elevator/Slot0/S", ElevatorConstants.k0S);
  private static final LoggedTunableNumber k0V = new LoggedTunableNumber("Elevator/Slot0/V", ElevatorConstants.k0V);
  private static final LoggedTunableNumber k0A = new LoggedTunableNumber("Elevator/Slot0/A", ElevatorConstants.k0A);
  private static final LoggedTunableNumber k0G = new LoggedTunableNumber("Elevator/Slot0/G", ElevatorConstants.k0G);

  private static final LoggedTunableNumber k1P = new LoggedTunableNumber("Elevator/Slot1/P", ElevatorConstants.k1P);
  private static final LoggedTunableNumber k1I = new LoggedTunableNumber("Elevator/Slot1/I", ElevatorConstants.k1I);
  private static final LoggedTunableNumber k1D = new LoggedTunableNumber("Elevator/Slot1/D", ElevatorConstants.k1D);
  private static final LoggedTunableNumber k1MaxV = new LoggedTunableNumber("Elevator/Slot1/MaxV", ElevatorConstants.k1MaxVelocity);
  private static final LoggedTunableNumber k1MaxA = new LoggedTunableNumber("Elevator/Slot1/MaxA", ElevatorConstants.k1MaxAcceleration);
  private static final LoggedTunableNumber k1S = new LoggedTunableNumber("Elevator/Slot1/S", ElevatorConstants.k1S);
  private static final LoggedTunableNumber k1V = new LoggedTunableNumber("Elevator/Slot1/V", ElevatorConstants.k1V);
  private static final LoggedTunableNumber k1A = new LoggedTunableNumber("Elevator/Slot1/A", ElevatorConstants.k1A);
  private static final LoggedTunableNumber k1G = new LoggedTunableNumber("Elevator/Slot0/G", ElevatorConstants.k1G);

  private static final LoggedTunableNumber k2P = new LoggedTunableNumber("Elevator/Slot2/P", ElevatorConstants.k2P);
  private static final LoggedTunableNumber k2I = new LoggedTunableNumber("Elevator/Slot2/I", ElevatorConstants.k2I);
  private static final LoggedTunableNumber k2D = new LoggedTunableNumber("Elevator/Slot2/D", ElevatorConstants.k2D);
  private static final LoggedTunableNumber k2MaxV = new LoggedTunableNumber("Elevator/Slot2/MaxV", ElevatorConstants.k2MaxVelocity);
  private static final LoggedTunableNumber k2MaxA = new LoggedTunableNumber("Elevator/Slot2/MaxA", ElevatorConstants.k2MaxAcceleration);
  private static final LoggedTunableNumber k2S = new LoggedTunableNumber("Elevator/Slot2/S", ElevatorConstants.k2S);
  private static final LoggedTunableNumber k2V = new LoggedTunableNumber("Elevator/Slot2/V", ElevatorConstants.k2V);
  private static final LoggedTunableNumber k2A = new LoggedTunableNumber("Elevator/Slot2/A", ElevatorConstants.k2A);
  private static final LoggedTunableNumber k2G = new LoggedTunableNumber("Elevator/Slot2/G", ElevatorConstants.k2G);

  private final SparkMax mElevatorSparkMax;
  private final ProfiledPIDController mElevatorProfiledPID;
  private ElevatorFeedforward mElevatorFF;

  private final RelativeEncoder mEncoder;

  public ElevatorSubsystem() {
    this.mElevatorSparkMax = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    this.mEncoder = mElevatorSparkMax.getEncoder();
    this.mElevatorProfiledPID = new ProfiledPIDController(
        ElevatorConstants.k0P,
        ElevatorConstants.k0I,
        ElevatorConstants.k0D,
        new Constraints(ElevatorConstants.k0MaxVelocity, ElevatorConstants.k0MaxAcceleration));
    this.mElevatorProfiledPID.setTolerance(ElevatorConstants.k0Tolerance);
    this.mElevatorFF = new ElevatorFeedforward(
        ElevatorConstants.k0S, ElevatorConstants.k0G, ElevatorConstants.k0V, ElevatorConstants.k0A);

    this.setDefaultCommand(enableFFCmd());

    this.mElevatorSparkMax.configure(
        ElevatorConstants.kElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public FunctionalCommand enableFFCmd() {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          double calculatedOutput = mElevatorFF.calculate(0);
          setVolts(calculatedOutput);
        },
        (interrupted) -> setVolts(mElevatorFF.calculate(0)),
        () -> false,
        this);
  }

  public boolean isPIDAtGoal() {
    return mElevatorProfiledPID.atGoal();
  }

  public FunctionalCommand coralLevelToPIDCmd(CoralLevel pCoralLevel) {
    ElevatorConstants.Setpoints elevatorSetpoint = ElevatorConstants.Setpoints.L1;

    if(pCoralLevel == CoralLevel.B3) {
      elevatorSetpoint = ElevatorConstants.Setpoints.L4;
    } else
    if(pCoralLevel == CoralLevel.B2) {
      elevatorSetpoint = ElevatorConstants.Setpoints.L3;
    } else
    if(pCoralLevel == CoralLevel.B1) {
      elevatorSetpoint = ElevatorConstants.Setpoints.L2;
    } 

    return setPIDCmd(elevatorSetpoint);
  }

  public FunctionalCommand setPIDCmd(ElevatorConstants.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          mElevatorProfiledPID.reset(getEncReading());
          mElevatorProfiledPID.setGoal(pSetpoint.getPos());
          SmartDashboard.putNumber("Elevator/Setpoint", pSetpoint.getPos());
        },
        () -> {
          double encoderReading = getEncReading();
          double calculatedPID = mElevatorFF.calculate(mElevatorProfiledPID.getSetpoint().velocity);
          double calculatedFF = mElevatorProfiledPID.calculate(encoderReading);
          setVolts(calculatedPID + calculatedFF);
        },
        (interrupted) -> setVolts(mElevatorFF.calculate(0.0)),
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

  public FunctionalCommand setTunablePIDCommand() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          double encoderReading = getEncReading();
          double calculatedPID = mElevatorFF.calculate(mElevatorProfiledPID.getSetpoint().velocity);
          double calculatedFF = mElevatorProfiledPID.calculate(encoderReading);

          setVolts(calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Elevator/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Elevator/PID Output", calculatedPID);
          SmartDashboard.putNumber("Elevator/FF Output", calculatedFF);
        },
        (interrupted) -> setVolts(0),
        () -> false,
        this);
  }

  public void setVolts(double pVoltage) {
    mElevatorSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12, 12.0));
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

  public Command setSlotCommand(int slot) {
    return new InstantCommand(() -> {
      this.slot = slot;
      switch(this.slot) {
        case 0:
          updatePIDandFF(k0P.get(), k0I.get(), k0D.get(), k0MaxV.get(), k0MaxA.get(), k0S.get(), k0V.get(), k0A.get(), k0G.get());
          break;
        case 1:
          updatePIDandFF(k1P.get(), k1I.get(), k1D.get(), k1MaxV.get(), k1MaxA.get(), k1S.get(), k1V.get(), k1A.get(), k1G.get());
          break;
        case 2:
          updatePIDandFF(k2P.get(), k2I.get(), k2D.get(), k2MaxV.get(), k2MaxA.get(), k2S.get(), k2V.get(), k2A.get(), k2G.get());
          break;
        default:
          Logger.recordOutput("STOP DUMBAHH", "ELEVATOR");
      }
    });
  }

  @Override
  public void periodic() {
    switch(slot) {
      case 0:
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
          updatePIDandFF(k0P.get(), k0I.get(), k0D.get(), k0MaxV.get(), k0MaxA.get(), k0S.get(), k0V.get(), k0A.get(), k0G.get());
        }, k0P, k0I, k0D, k0MaxV, k0MaxA, k0S, k0V, k0A, k0G);
        break;
      case 1:
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
          updatePIDandFF(k1P.get(), k1I.get(), k1D.get(), k1MaxV.get(), k1MaxA.get(), k1S.get(), k1V.get(), k1A.get(), k1G.get());
        }, k1P, k1I, k1D, k1MaxV, k1MaxA, k1S, k1V, k1A, k1G);
        break;
      case 2:
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
          updatePIDandFF(k2P.get(), k2I.get(), k2D.get(), k2MaxV.get(), k2MaxA.get(), k2S.get(), k2V.get(), k2A.get(), k2G.get());
        }, k2P, k2I, k2D, k2MaxV, k2MaxA, k2S, k2V, k2A, k2G);
        break;
      default:
        Logger.recordOutput("STOP DUMBAHH", "ELEVATOR");
    }

    Logger.recordOutput("Elevator/kP", mElevatorProfiledPID.getP());
    Logger.recordOutput("Elevator/kI", mElevatorProfiledPID.getI());
    Logger.recordOutput("Elevator/kD", mElevatorProfiledPID.getD());
    Logger.recordOutput("Elevator/kMaxV", mElevatorProfiledPID.getConstraints().maxVelocity);
    Logger.recordOutput("Elevator/kMaxA", mElevatorProfiledPID.getConstraints().maxAcceleration);
    Logger.recordOutput("Elevator/kS", mElevatorFF.getKs());
    Logger.recordOutput("Elevator/kV", mElevatorFF.getKv());
    Logger.recordOutput("Elevator/kA", mElevatorFF.getKa());
    Logger.recordOutput("Elevator/kG", mElevatorFF.getKg());

    stopIfLimit();

    SmartDashboard.putNumber("Elevator/Position", getEncReading());
    // SmartDashboard.putNumber("Elevator/Velocity", mEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator/Output", getMotorOutput());
    SmartDashboard.putNumber("Elevator/Voltage", mElevatorSparkMax.getBusVoltage());
    SmartDashboard.putBoolean("Elevator/At Setpoint", isPIDAtGoal());
  }

  public void updatePIDandFF(double kP, double kI, double kD, double kMaxV, double kMaxA, double kS, double kV, double kA, double kG) {
    mElevatorProfiledPID.setPID(kP, kI, kD);
    mElevatorProfiledPID.setConstraints(new Constraints(kMaxV, kMaxA));
    mElevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
  }
}
