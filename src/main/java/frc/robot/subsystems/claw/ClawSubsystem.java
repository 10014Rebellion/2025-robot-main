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

public class ClawSubsystem extends SubsystemBase {
  private final SparkFlex mClawSparkMax;
  private final DigitalInput mBeamBreak;

  public ClawSubsystem() {
    this.mClawSparkMax = new SparkFlex(ClawConstants.kClawID, ClawConstants.kMotorType);
    this.mBeamBreak = new DigitalInput(ClawConstants.kBeamBreakDIOPort);

    // this.setDefaultCommand(holdCoralCmd());

    mClawSparkMax.configure(
        ClawConstants.kClawConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void setClaw(ClawConstants.RollerSpeed pVoltage) {
    setClaw(pVoltage.get());
  }

  public void setClaw(double pVoltage) {
    mClawSparkMax.setVoltage(pVoltage); // MathUtil.clamp(pVoltage, -12, 12));
  }

  public FunctionalCommand setClawCmd(double pVoltage) {
    return new FunctionalCommand(
        () -> setClaw(pVoltage), () -> setClaw(pVoltage), (interrupted) -> {}, () -> false, this);
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
        () ->
            setClaw(
                ClawConstants.RollerSpeed
                    .INTAKE_CORAL), // Start the command by setting the claw to coral speed
        () -> {
          // if (; // igetBeamBreak()) setClaw(0.0)f we detect a coral, stop the roller
          setClaw(ClawConstants.RollerSpeed.INTAKE_CORAL); // if we dont, try and intake
        },
        (interrupted) -> setClaw(ClawConstants.RollerSpeed.HOLD_CORAL),
        () -> getBeamBreak(),
        this);
  }

  public FunctionalCommand scoreCoralCmd(ClawConstants.RollerSpeed speed) {
    return new FunctionalCommand(
        () -> setClaw(speed),
        () -> {
          setClaw(speed);
        },
        (interrupted) -> setClaw(speed),
        () -> !getBeamBreak(),
        this);
  }

  public FunctionalCommand scoreCoralCmd() {
    return new FunctionalCommand(
        () -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
        () -> {
          setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF);
        },
        (interrupted) -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
        () -> !getBeamBreak(),
        this);
  }

  public FunctionalCommand scoreReverseCoralCmd() {
    return new FunctionalCommand(
        () -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
        () -> {
          setClaw(ClawConstants.RollerSpeed.EJECT_CORAL);
        },
        (interrupted) -> setClaw(ClawConstants.RollerSpeed.EJECT_CORAL),
        () -> !getBeamBreak(),
        this);
  }

  // returns true when beambreak is broken (Coral is in the claw)
  // returns false when beambreak is intact (Coral is not in the claw)
  public boolean getBeamBreak() {
    return !mBeamBreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Claw/Beam Break", getBeamBreak());
    SmartDashboard.putNumber("Claw/AppliedOutput (Volts)", mClawSparkMax.getAppliedOutput() * 12.0);
  }
}
