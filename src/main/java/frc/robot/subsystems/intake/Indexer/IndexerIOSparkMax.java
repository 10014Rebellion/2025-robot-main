package frc.robot.subsystems.intake.Indexer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.intake.Indexer.IndexerConstants.IndexerConfiguration;
import frc.robot.subsystems.intake.Indexer.IndexerConstants.IndexerHardware;

public class IndexerIOSparkMax implements IndexerIO{

    private final SparkFlex kMotor;
    private SparkFlexConfig motorConfiguration = new SparkFlexConfig();
    
    private IndexerConfiguration indexerConfiguration;

    public IndexerIOSparkMax(IndexerHardware hardware, IndexerConfiguration indexerConfiguration){
        kMotor = new SparkFlex(hardware.kMotorPort(), hardware.kMotorType());
        this.indexerConfiguration = indexerConfiguration;

        motorConfiguration.inverted(indexerConfiguration.kInverted());
        motorConfiguration.idleMode(indexerConfiguration.kIdleMode());
        motorConfiguration.smartCurrentLimit(indexerConfiguration.kSmartLimit());
        motorConfiguration.secondaryCurrentLimit(indexerConfiguration.kSecondaryLimit());

        kMotor.configure(
            motorConfiguration, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs){
        // Setting this to true since we don't care about this value
        inputs.isMotorConnected = true;

        inputs.appliedVoltage = kMotor.getAppliedOutput() * kMotor.getBusVoltage();
        inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = kMotor.getMotorTemperature();
        inputs.motorOutput = kMotor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double pVolts){
        pVolts = MathUtil.clamp(pVolts, -12.0, 12.0);
        kMotor.setVoltage(pVolts);
    }

    @Override
    public void stop(){
        kMotor.stopMotor();
    }

    
}
