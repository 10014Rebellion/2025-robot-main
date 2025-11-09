package frc.robot.subsystems.climb.pulley;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climb.pulley.PulleyConstants.Pulley.EncoderConfiguration;
import frc.robot.subsystems.climb.pulley.PulleyConstants.Pulley.PulleyConfiguration;
import frc.robot.subsystems.climb.pulley.PulleyConstants.Pulley.PulleyHardware;

public class PulleyIOSparkMax implements PulleyIO{

    private final SparkMax kMotor;
    private final AbsoluteEncoder kEncoder;

    private SparkMaxConfig motorConfiguration = new SparkMaxConfig();

    public PulleyIOSparkMax(PulleyHardware hardware, PulleyConfiguration pulleyConfig, EncoderConfiguration encoderConfig) {
        kMotor = new SparkMax(hardware.kMotorID(), hardware.kMotorType());
        kEncoder = kMotor.getAbsoluteEncoder();

        motorConfiguration.inverted(pulleyConfig.kInverted());
        motorConfiguration.idleMode(pulleyConfig.kIdleMode());
        motorConfiguration.smartCurrentLimit(pulleyConfig.kSmartLimit());
        motorConfiguration.secondaryCurrentLimit(pulleyConfig.kSecondaryLimit());

        motorConfiguration
            .absoluteEncoder
            // .inverted(encoderConfig.kInverted())
            .zeroOffset(encoderConfig.offsetRev())
            .positionConversionFactor(encoderConfig.kPosistionFactor())
            .velocityConversionFactor(encoderConfig.kVelocityFactor());

        kMotor.configure(
            motorConfiguration, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);


    }

    @Override
    public void updateInputs(PulleyIOInputs inputs) {
        // Setting this to true since we don't care about this value
        inputs.isMotorConnected = true;

        inputs.appliedVoltage = kMotor.getAppliedOutput() * kMotor.getBusVoltage();
        inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = kMotor.getMotorTemperature();
        inputs.posistionDegrees = getPulleyPosition();
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

    private double getPulleyPosition() {
        double encoderMeasurement = kEncoder.getPosition();
        if (encoderMeasurement > 180.0)
          encoderMeasurement -= 360.0;
        return encoderMeasurement;
    }

    @AutoLogOutput(key="Climb/Pulley/MotorOutput")
    public double getMotorOutput(){
        return kMotor.getAppliedOutput();
    }

    
}