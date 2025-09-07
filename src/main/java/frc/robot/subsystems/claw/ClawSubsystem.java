// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.controls.StateTracker.GamePiece;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ClawSubsystem extends SubsystemBase {
  private final SparkFlex mClawSparkMax;
  private final DigitalInput mBeamBreak;

  private final CANrange distanceFromClawArcSensor;

  private final StatusSignal<Distance> distanceFromClaw;
  private final StatusSignal<Boolean> isDetected;
  private final StatusSignal<Distance> distanceStdDevClaw;
  private final StatusSignal<Double> ambience;
  private final StatusSignal<Double> signalStrength;
  private final StatusSignal<MeasurementHealthValue> measurementHealth;
  private final StatusSignal<Time> measurementTime;

  public ClawSubsystem() {
    this.mClawSparkMax = new SparkFlex(ClawConstants.kClawID, ClawConstants.kMotorType);
    this.mBeamBreak = new DigitalInput(ClawConstants.kBeamBreakDIOPort);

    this.distanceFromClawArcSensor = new CANrange(ClawConstants.CANRangeID, Constants.kCanbusName);
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 3000;
    config.ProximityParams.ProximityHysteresis = 0.01;
    config.ProximityParams.ProximityThreshold = 0.5;

    distanceFromClawArcSensor.getConfigurator().apply(config);

    distanceFromClaw = distanceFromClawArcSensor.getDistance();
    isDetected = distanceFromClawArcSensor.getIsDetected();
    distanceStdDevClaw = distanceFromClawArcSensor.getDistanceStdDev();
    ambience = distanceFromClawArcSensor.getAmbientSignal();

    signalStrength = distanceFromClawArcSensor.getSignalStrength();
    measurementHealth = distanceFromClawArcSensor.getMeasurementHealth();
    measurementTime = distanceFromClawArcSensor.getMeasurementTime();

    mClawSparkMax.configure(
        ClawConstants.kClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClaw(ClawConstants.RollerSpeed pVoltage) {
    setClaw(pVoltage.get());
  }

  public void setClaw(double pVoltage) {
    mClawSparkMax.setVoltage(pVoltage);
  }

  public FunctionalCommand setClawCmd(double pVoltage) {
    return new FunctionalCommand(
        () -> setClaw(pVoltage), () -> setClaw(pVoltage), (interrupted) -> {
        }, () -> false, this);
  }

  public FunctionalCommand holdCoralCmd() {
    return new FunctionalCommand(
        () -> setClaw(ClawConstants.RollerSpeed.HOLD_CORAL),
        () -> setClaw(ClawConstants.RollerSpeed.HOLD_CORAL),
        (interrupted) -> setClaw(0.0),
        () -> false,
        this);
  }

  public FunctionalCommand intakeCoralCmd() {
    return new FunctionalCommand(
        () -> setClaw(
            ClawConstants.RollerSpeed.INTAKE_CORAL),
        () -> {
          setClaw(ClawConstants.RollerSpeed.INTAKE_CORAL);
        },
        (interrupted) -> setClaw(ClawConstants.RollerSpeed.HOLD_CORAL),
        () -> hasPiece(),
        this);
  }

  public FunctionalCommand scoreCoralCmd(ClawConstants.RollerSpeed speed) {
    return new FunctionalCommand(
        () -> setClaw(speed),
        () -> {
          setClaw(speed);
        },
        (interrupted) -> setClaw(speed),
        () -> !hasPiece(),
        this);
  }

  public FunctionalCommand scoreCoralCmd() {
    return new FunctionalCommand(
        () -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
        () -> {
          setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF);
        },
        (interrupted) -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
        () -> !hasPiece(),
        this);
  }

  public FunctionalCommand scoreReverseCoralCmd() {
    return new FunctionalCommand(
        () -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
        () -> {
          setClaw(ClawConstants.RollerSpeed.EJECT_CORAL);
        },
        (interrupted) -> setClaw(ClawConstants.RollerSpeed.EJECT_CORAL),
        () -> !hasPiece(),
        this);
  }

  public FunctionalCommand groundAlgae() {
    return new FunctionalCommand(
        () -> setClaw(ClawConstants.RollerSpeed.GROUND_ALGAE),
        () -> {
          setClaw(ClawConstants.RollerSpeed.GROUND_ALGAE);
        },
        (interrupted) -> setClaw(ClawConstants.RollerSpeed.HOLD_ALGAE),
        () -> !hasPiece(),
        this);
  }

  public FunctionalCommand throwAlgae(WristSubsystem mWrist, ElevatorSubsystem mElevator) {
    return new FunctionalCommand(
        () -> setClaw(ClawConstants.RollerSpeed.HOLD_ALGAE),
        () -> {
          if (mWrist.getEncReading() >= WristConstants.throwAlgaePos)
            setClaw(ClawConstants.RollerSpeed.SCORE_BARGE);
        },
        (interrupted) -> {
          if (mWrist.getEncReading() >= WristConstants.throwAlgaePos)
            setClaw(ClawConstants.RollerSpeed.SCORE_BARGE);
        },
        () -> mWrist.getEncReading() >= WristConstants.throwAlgaePos,
        this);
  }

  public FunctionalCommand autonThrowAlgae(WristSubsystem mWrist, ElevatorSubsystem mElevator) {
    return new FunctionalCommand(
        () -> setClaw(ClawConstants.RollerSpeed.HOLD_ALGAE),
        () -> {
          if (mWrist.getEncReading() >= WristConstants.throwAlgaePos)
            setClaw(ClawConstants.RollerSpeed.SCORE_BARGE);
        },
        (interrupted) -> {
          if (mWrist.getEncReading() >= WristConstants.throwAlgaePos)
            setClaw(ClawConstants.RollerSpeed.SCORE_BARGE);
        },
        () -> (mWrist.getEncReading() > WristConstants.throwAlgaePos + 6),
        this);
  }

  // TODO: CHEK FOR BEAMBREAK AFTER ITS FIXED BY BOSCO (fricken bosco.....)
  public boolean hasPiece() {
    return CANRangeHasPiece();
  }

  public boolean beambreakHasPiece() {
    return !mBeamBreak.get();
  }

  public boolean CANRangeHasPiece() {
    return (distanceFromClaw.getValueAsDouble() < 0.08) && 
    (signalStrength.getValueAsDouble() > 3000) &&
    (ambience.getValueAsDouble() < 20);
  }

  public GamePiece CANRangeGuessGamePiece() {
    if((distanceFromClaw.getValueAsDouble() < 0.08) && 
      (signalStrength.getValueAsDouble() > 3000) && 
      (signalStrength.getValueAsDouble() < 20000) &&
      (ambience.getValueAsDouble() < 20)) return GamePiece.Algae;
    
    if((distanceFromClaw.getValueAsDouble() < ClawConstants.coralDetectionCutoff) && 
    (signalStrength.getValueAsDouble() > 20000)  && 
    (ambience.getValueAsDouble() < 20)) return GamePiece.Coral;

    return null;
  }


  @Override
  public void periodic() {
    Logger.recordOutput("Claw/Has Piece", hasPiece());
    Logger.recordOutput("Claw/Beambreak/Has Piece", beambreakHasPiece());
    Logger.recordOutput("Claw/AppliedOutput (Volts)", mClawSparkMax.getAppliedOutput() * mClawSparkMax.getBusVoltage());
    Logger.recordOutput("Claw/CANRange/Connected", BaseStatusSignal.refreshAll(
      distanceFromClaw,
      distanceStdDevClaw,
      isDetected,
      ambience,
      signalStrength,
      measurementHealth,
      measurementTime
    ).isOK());
    
    Logger.recordOutput("Claw/CANRange/DistanceFromClaw", distanceFromClaw.getValueAsDouble());
    Logger.recordOutput("Claw/CANRange/DistanceFromClawStddev", distanceStdDevClaw.getValueAsDouble());
    Logger.recordOutput("Claw/CANRange/IsDetected", isDetected.getValue());
    Logger.recordOutput("Claw/CANRange/Ambience", ambience.getValueAsDouble());
    Logger.recordOutput("Claw/CANRange/SignalStrength", signalStrength.getValueAsDouble());
    Logger.recordOutput("Claw/CANRange/MeasurementHealthBool", measurementHealth.getValue().equals(MeasurementHealthValue.Good));
    Logger.recordOutput("Claw/CANRange/MeasurementTime", measurementTime.getValueAsDouble());
    Logger.recordOutput("Claw/CANRange/Detects Piece", CANRangeHasPiece());


  }
}
