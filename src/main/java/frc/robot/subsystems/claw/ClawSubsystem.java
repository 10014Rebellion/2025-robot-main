// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

public class ClawSubsystem extends SubsystemBase {
  private final SparkFlex mLeftClawSparkMax;
  private final SparkFlex mRightClawSparkMax;

  private final DutyCycleEncoder mClawEncoder;

  public ClawSubsystem() {
    this.mLeftClawSparkMax =
        new SparkFlex(ClawConstants.kLeftClawID, ClawConstants.kMotorType);
    this.mRightClawSparkMax =
        new SparkFlex(ClawConstants.kRightClawID, ClawConstants.kMotorType);
    this.mClawEncoder = new DutyCycleEncoder(ClawConstants.kEncoderDIOPort);

    mLeftClawSparkMax.configure(
        ClawConstants.kClawConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    mRightClawSparkMax.configure(
        ClawConstants.kClawConfig.inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void setClaw(ClawConstants.Setpoints pVoltage) {
    setClaw(pVoltage.get());
  }

  public void setClaw(double pVoltage) {
    mLeftClawSparkMax.setVoltage(MathUtil.clamp(pVoltage, -12, 12));
    mRightClawSparkMax.setVoltage(MathUtil.clamp(pVoltage, -12, 12));
  }

  public double getClaw() {
    double measurement = (mClawEncoder.get() * 360.0) + ClawConstants.kEncoderOffset;
    if (measurement >= 180) {
      return measurement - 360;
    }
    return measurement;
  }

  public boolean hasCoral() {
    return Math.abs(getClaw() - ClawConstants.ClawOpenPositions.HAS_CORAL.get())
        < ClawConstants.positionTolerance;
  }

  public boolean isClawOpen() {
    return getClaw() > ClawConstants.ClawOpenPositions.OPEN.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw/Open Value", getClaw());
    SmartDashboard.putBoolean("Claw/Periodic Has Coral", hasCoral());
  }
}
