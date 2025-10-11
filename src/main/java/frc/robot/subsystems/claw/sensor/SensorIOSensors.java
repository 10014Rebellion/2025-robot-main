package frc.robot.subsystems.claw.sensor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.MeasurementHealthValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.claw.sensor.SensorConstants.ClawSensorsConfiguration;

public class SensorIOSensors implements SensorIO{

    private final CANrange kCANRangeClaw;
    private final DigitalInput kBeamBreak;

    private final StatusSignal<Distance> distanceFromClaw;
    private final StatusSignal<Boolean> isDetected;
    private final StatusSignal<Distance> distanceStdDevClaw;
    private final StatusSignal<Double> ambience;
    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<MeasurementHealthValue> measurementHealth;
    private final StatusSignal<Time> measurementTime;

    private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

    public SensorIOSensors(ClawSensorsConfiguration configuration){
        kCANRangeClaw = new CANrange(configuration.CANRangeID(), Constants.kCanbusName);
        kBeamBreak = new DigitalInput(configuration.beamBreakPort());

        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 3000;
        canRangeConfig.ProximityParams.ProximityHysteresis = 0.01;
        canRangeConfig.ProximityParams.ProximityThreshold = 0.5;

        kCANRangeClaw.getConfigurator().apply(canRangeConfig);

        // Intialize Status Signals
        distanceFromClaw = kCANRangeClaw.getDistance();
        isDetected = kCANRangeClaw.getIsDetected();
        distanceStdDevClaw = kCANRangeClaw.getDistanceStdDev();
        ambience = kCANRangeClaw.getAmbientSignal();
        signalStrength = kCANRangeClaw.getSignalStrength();
        measurementHealth = kCANRangeClaw.getMeasurementHealth();
        measurementTime = kCANRangeClaw.getMeasurementTime();
    }

    @SuppressWarnings("unlikely-arg-type")
    @Override
    public void updateInputs(SensorIOInputs inputs){
        inputs.distanceFromClaw = distanceFromClaw.getValueAsDouble();
        inputs.distanceFromClawStddev = distanceStdDevClaw.getValueAsDouble();
        inputs.isCANRangeDetected = isDetected.getValue();
        inputs.isBeamBreakDetected = getBeamBreakValue();
        inputs.ambience = ambience.getValueAsDouble();
        inputs.signalStrength = signalStrength.getValueAsDouble();
        inputs.isMeasurementHealthGood = measurementHealth.equals(MeasurementHealthValue.Good);
        inputs.measurementTime = measurementTime.getValueAsDouble();
        
        inputs.isSensorConnected = BaseStatusSignal.refreshAll(
            distanceFromClaw,
            distanceStdDevClaw,
            isDetected,
            ambience,
            signalStrength,
            measurementHealth,
            measurementTime).isOK();
    }

    @Override
    public boolean getBeamBreakValue(){
        return !kBeamBreak.get();
    }

    @Override
    public boolean getCANRangeValue(){
        return isDetected.getValue();
    }
    
}
