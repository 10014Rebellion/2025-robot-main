package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.wrist.WristConstants.EncoderHardware;
import frc.robot.subsystems.wrist.WristConstants.WristConfiguration;
import frc.robot.subsystems.wrist.WristConstants.WristHardware;

    
public class WristIOSparkMax implements WristIO{

    private final SparkMax kMotor;
    private final DutyCycleEncoder kEncoder;
    private SparkMaxConfig motorConfiguration = new SparkMaxConfig();

    private WristConfiguration wristConfiguration;
    public WristIOSparkMax(WristHardware pHardware, WristConfiguration pWristConfiguration, EncoderHardware pEncoderConfiguration){
        
        kMotor = new SparkMax(pHardware.kMotorPort(), pHardware.kMotorType());
        
        this.wristConfiguration = pWristConfiguration;
        EncoderHardware encoderConfiguration = pEncoderConfiguration;

        kEncoder = new DutyCycleEncoder(encoderConfiguration.kEncoderPort());

        motorConfiguration.inverted(pWristConfiguration.kInverted());
        motorConfiguration.idleMode(pWristConfiguration.kIdleMode());
        motorConfiguration.smartCurrentLimit(pWristConfiguration.kSmartLimit());
        motorConfiguration.secondaryCurrentLimit(pWristConfiguration.kSecondaryLimit());

        motorConfiguration.absoluteEncoder.positionConversionFactor(1);
        motorConfiguration.absoluteEncoder.velocityConversionFactor(1);
        motorConfiguration.absoluteEncoder.zeroOffset(0);

        kEncoder.setDutyCycleRange(0, 1);
        kEncoder.setInverted(WristConstants.kEncoderInverted);

        kMotor.configure(
            motorConfiguration, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        // Setting this to true since we don't care about this value
        inputs.isMotorConnected = true;

        inputs.appliedVoltage = kMotor.getAppliedOutput() * kMotor.getBusVoltage();
        inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = kMotor.getMotorTemperature();
        inputs.positionMeters = getPosition();
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

    public double getPosition() {
        double encoderMeasurement = (getRawPosition() * WristConstants.kPositionConversionFactor)
            + WristConstants.kEncoderOffsetDeg;
        if (encoderMeasurement > WristConstants.kPositionConversionFactor / 2.0)
          encoderMeasurement -= WristConstants.kPositionConversionFactor;
        return encoderMeasurement;
    }

    public double getRawPosition(){
        return kEncoder.get();
    }

    public double getMotorOutput(){
        return kMotor.getAppliedOutput();
    }

    public double shouldApplyVolts(double pVolts){
        if ((pVolts < 0 ) && (getPosition() <= wristConfiguration.kReverseSoftLimit())) {
            return 0;
        }

        if ((pVolts > 0) && (getPosition() >= wristConfiguration.kForwardSoftLimit()) ){
            return 0;
        }

        else{
            return pVolts;
        }
    }

    public void stopElevatorIfLimit(double pVolts){
        if( (getPosition() > wristConfiguration.kForwardSoftLimit()) && (getMotorOutput() > 0)){
            setVoltage(0);
        }

        else if( (getPosition() < wristConfiguration.kReverseSoftLimit()) && (getMotorOutput() < 0)){
            setVoltage(0);
        }
        
    }
    
}

