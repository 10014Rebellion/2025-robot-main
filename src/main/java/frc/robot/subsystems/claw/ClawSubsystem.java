// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  private final SparkFlex mClawSparkMax;

  public ClawSubsystem() {
    this.mClawSparkMax = new SparkFlex(ClawConstants.kClawID, ClawConstants.kMotorType);

    mClawSparkMax.configure(
        ClawConstants.kClawConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void setClaw(ClawConstants.Setpoints pVoltage) {
    setClaw(pVoltage.get());
  }

  public void setClaw(double pVoltage) {
    mClawSparkMax.setVoltage(MathUtil.clamp(pVoltage, -12, 12));
  }

  @Override
  public void periodic() {}

  public Command setClawCmd(double pVoltage) {
    return new InstantCommand(() -> setClaw(pVoltage));
  }
}
