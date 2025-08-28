// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ClawSubsystem extends SubsystemBase {
  private final SparkFlex mClawSparkMax;
  private final DigitalInput mBeamBreak;

  public ClawSubsystem() {
    this.mClawSparkMax = new SparkFlex(ClawConstants.kClawID, ClawConstants.kMotorType);
    this.mBeamBreak = new DigitalInput(ClawConstants.kBeamBreakDIOPort);

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

  public boolean hasPiece() {
    return !mBeamBreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Claw/Beam Break", hasPiece());
    SmartDashboard.putNumber("Claw/AppliedOutput (Volts)", mClawSparkMax.getAppliedOutput() * 12.0);
  }
}
