package frc.robot.subsystems.intake.IntakePivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.intake.IntakePivot.IntakePivotConstants.IntakePivotConfiguration;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotConstants.IntakePivotEncoderHardware;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotConstants.IntakePivotHardware;
import frc.robot.subsystems.wrist.WristConstants;

public class IntakePivotIOSparkMax implements IntakePivotIO{

    private final SparkMax kMotor;
    private final AbsoluteEncoder kEncoder;

    private SparkMaxConfig motorConfiguration = new SparkMaxConfig();
    private IntakePivotConfiguration intakePivotConfiguration;

    public IntakePivotIOSparkMax(IntakePivotHardware hardware, IntakePivotConfiguration indexerConfiguration, IntakePivotEncoderHardware encoderConfiguration){
        kMotor = new SparkMax(hardware.kMotorPort(), hardware.kMotorType());
        kEncoder = kMotor.getAbsoluteEncoder();
        
        motorConfiguration.inverted(indexerConfiguration.kInverted());
        motorConfiguration.idleMode(indexerConfiguration.kIdleMode());
        motorConfiguration.smartCurrentLimit(indexerConfiguration.kSmartLimit());
        motorConfiguration.secondaryCurrentLimit(indexerConfiguration.kSecondaryLimit());

        motorConfiguration
            .absoluteEncoder
            .positionConversionFactor(indexerConfiguration.kPositionConversionFactor())
            .velocityConversionFactor(indexerConfiguration.kVelocityConversionFactor())
            .inverted(encoderConfiguration.kEncoderInverted())
            .zeroOffset(encoderConfiguration.kEncoderOffset());

        kMotor.configure(
            motorConfiguration, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }


    @Override
    public void updateInputs(IntakePivotIOInputs inputs){
        // Setting this to true since we don't care about this value
        inputs.isMotorConnected = true;

        inputs.appliedVoltage = kMotor.getAppliedOutput() * kMotor.getBusVoltage();
        inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = kMotor.getMotorTemperature();
        inputs.motorOutput = kMotor.getAppliedOutput();
        inputs.position = getPosition();
    }

    public double getPosition(){
        double measurement = kEncoder.getPosition();
        if (measurement >= 180) {
          return measurement - 360;
        }
        return measurement;
    }

    @Override
    public void setVoltage(double pVolts){
        pVolts = filterToLimitsIntakePivot(MathUtil.clamp(pVolts, -12.0, 12.0));
        kMotor.setVoltage(pVolts);
    }

    private boolean isOutOfBoundsIntakePivot(double pInput) {
        return (pInput > 0 && getPosition() >= WristConstants.kForwardSoftLimit)
            || (pInput < 0 && getPosition() <= WristConstants.kReverseSoftLimit);
    }

    private double filterToLimitsIntakePivot(double pInput) {
        return isOutOfBoundsIntakePivot(pInput) ? 0.0 : pInput;
      }
    
    public void stopIfLimitIntakePivot() {
        if (isOutOfBoundsIntakePivot(kMotor.getAppliedOutput())) {
            setVoltage(0);
        }
    }

    @Override
    public void stop(){
        kMotor.stopMotor();
    }
    
}
