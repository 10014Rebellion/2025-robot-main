package frc.robot.util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

public class ConfigMotor {
    public static void configSparkMax(SparkMax motor, boolean motorInverted, int currentLimit, IdleMode idleMode, RelativeEncoder encoder, double conversionFactor, int CPR) {
        var motorConfig = new SparkMaxConfig();
        var encoderConfig = new EncoderConfig();

        encoderConfig
            .positionConversionFactor(conversionFactor)
            .countsPerRevolution(CPR);

        motorConfig
            .smartCurrentLimit(currentLimit)
            .inverted(motorInverted)
            .idleMode(idleMode)
            .apply(encoderConfig);
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);
    }
}
