package frc.robot.subsystems.claw.claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.claw.claw.ClawConstants.ControllerConfig;
import frc.robot.subsystems.claw.claw.ClawConstants.MotorConfiguration;

public class ClawIOSparkMax implements ClawIO {
  private final SparkFlex kMotor;
  private SparkFlexConfig kSparkMaxConfig = new SparkFlexConfig();

  public ClawIOSparkMax(MotorConfiguration motorConfiguration, ControllerConfig controllerConfig){
    kMotor = new SparkFlex(motorConfiguration.motorID(), motorConfiguration.motorType());

    kSparkMaxConfig.inverted(motorConfiguration.inverted());
    kSparkMaxConfig.idleMode(motorConfiguration.idleMode());
    kSparkMaxConfig.smartCurrentLimit(motorConfiguration.smartCurrentLimit());
    kSparkMaxConfig.secondaryCurrentLimit(motorConfiguration.secondaryCurrentLimit());

    kMotor.configure(kSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs){
    inputs.appliedVoltage = kMotor.getAppliedOutput() * kMotor.getBusVoltage();
    inputs.isMotorConnected = true;
    inputs.statorCurrentAmps = 0.0;
    inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
    inputs.temperatureCelsius = kMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts){
    kMotor.setVoltage(volts);
  }
    
}