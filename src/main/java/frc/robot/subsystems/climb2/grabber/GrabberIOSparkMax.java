package frc.robot.subsystems.climb2.grabber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.climb2.grabber.GrabberConstants.Grabber.GrabberConfiguration;
import frc.robot.subsystems.climb2.grabber.GrabberConstants.Grabber.GrabberHardware;

public class GrabberIOSparkMax implements GrabberIO{

    private final SparkMax kMotor;
    private final DigitalInput kClimbBeamBreak;

    private SparkMaxConfig motorConfiguration = new SparkMaxConfig();

    public GrabberIOSparkMax(GrabberHardware hardware, GrabberConfiguration configuration) {
        kMotor = new SparkMax(hardware.kMotorID(), hardware.kMotorType());
        kClimbBeamBreak = new DigitalInput(hardware.kBeamBreakID());

        motorConfiguration.inverted(configuration.inverted());
        motorConfiguration.idleMode(configuration.kIdleMode());
        motorConfiguration.smartCurrentLimit(configuration.kSmartLimit());
        motorConfiguration.secondaryCurrentLimit(configuration.kSecondaryLimit());

        kMotor.configure(
            motorConfiguration, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);


    }

    public boolean hasChain() {
        return !kClimbBeamBreak.get();
      }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        // Setting this to true since we don't care about this value
        inputs.isMotorConnected = true;

        inputs.appliedVoltage = kMotor.getAppliedOutput() * kMotor.getBusVoltage();
        inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = kMotor.getMotorTemperature();
        inputs.hasChain = hasChain();
    }

    @Override
    public void setVoltage(double volts) {
        volts = MathUtil.applyDeadband(volts, -12.0, 12.0);
        kMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        kMotor.stopMotor();
  }
    
}
