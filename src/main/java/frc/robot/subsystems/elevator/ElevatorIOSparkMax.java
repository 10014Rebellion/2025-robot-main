package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorConfiguration;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHardware;

import com.revrobotics.spark.SparkMax;

public class ElevatorIOSparkMax implements ElevatorIO{

    private final SparkMax kMotor;
    private final RelativeEncoder kEncoder;
    private SparkMaxConfig motorConfiguration = new SparkMaxConfig();

    private ElevatorConfiguration configuration;

    public ElevatorIOSparkMax(ElevatorHardware hardware, ElevatorConfiguration configuration){
        kMotor = new SparkMax(hardware.kMotorPort(), hardware.kMotorType());
        kEncoder = kMotor.getEncoder();
        this.configuration = configuration;

        motorConfiguration.inverted(configuration.kInverted());
        motorConfiguration.idleMode(configuration.kIdleMode());
        motorConfiguration.smartCurrentLimit(configuration.kSmartLimit());
        motorConfiguration.secondaryCurrentLimit(configuration.kSecondaryLimit());

        motorConfiguration.encoder.positionConversionFactor(1);
        motorConfiguration.encoder.velocityConversionFactor(1);

        kMotor.configure(
            motorConfiguration, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        // Setting this to true since we don't care about this value
        inputs.isMotorConnected = true;

        inputs.appliedVoltage = kMotor.getAppliedOutput() * kMotor.getBusVoltage();
        inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = kMotor.getMotorTemperature();
        inputs.positionMeters = getPosition();
        inputs.velocityMs = getVelocity();
        inputs.motorOutput = kMotor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double pVolts){
        pVolts = shouldApplyVolts(MathUtil.clamp(pVolts, -12.0, 12.0));
        kMotor.setVoltage(pVolts);
    }

    @Override
    public void stop(){
        kMotor.stopMotor();
    }

    public double getPosition(){
        return kEncoder.getPosition() * configuration.kPositionConversionFactor();
    }

    public double getVelocity(){
        return kEncoder.getVelocity() * configuration.kVelocityConversionFactor();
    }

    public double getMotorOutput(){
        return kMotor.getAppliedOutput();
    }


    public double shouldApplyVolts(double pVolts){
        if ((pVolts < 0 ) && (getPosition() <= configuration.kReverseSoftLimit())) {
            return 0;
        }

        if ((pVolts > 0) && (getPosition() >= configuration.kForwardSoftLimit()) ){
            return 0;
        }

        else{
            return pVolts;
        }
    }

    public void stopElevatorIfLimit(double pVolts){
        if( (getPosition() > configuration.kForwardSoftLimit()) && (getMotorOutput() > 0)){
            setVoltage(0);
        }

        else if( (getPosition() < configuration.kReverseSoftLimit()) && (getMotorOutput() < 0)){
            setVoltage(0);
        }
        
    }
    
}
