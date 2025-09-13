package frc.robot.subsystems.claw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.MeasurementHealthValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

public class CANRangeIO implements ToFIO {
    private final CANrange distanceFromClawArcSensor;

    private final StatusSignal<Distance> distanceFromClaw;
    private final StatusSignal<Boolean> isDetected;
    private final StatusSignal<Distance> distanceStdDevClaw;
    private final StatusSignal<Double> ambience;
    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<MeasurementHealthValue> measurementHealth;
    private final StatusSignal<Time> measurementTime;

    public CANRangeIO(int ID, String name) {
        this.distanceFromClawArcSensor = new CANrange(ClawConstants.CANRangeID, Constants.kCanbusName);
        CANrangeConfiguration config = new CANrangeConfiguration();
        config.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
        config.ProximityParams.ProximityHysteresis = 0.01;
        config.ProximityParams.ProximityThreshold = 0.5;

        distanceFromClawArcSensor.getConfigurator().apply(config);

        distanceFromClaw = distanceFromClawArcSensor.getDistance();
        isDetected = distanceFromClawArcSensor.getIsDetected();
        distanceStdDevClaw = distanceFromClawArcSensor.getDistanceStdDev();
        ambience = distanceFromClawArcSensor.getAmbientSignal();

        signalStrength = distanceFromClawArcSensor.getSignalStrength();
        measurementHealth = distanceFromClawArcSensor.getMeasurementHealth();
        measurementTime = distanceFromClawArcSensor.getMeasurementTime();
    }

    @Override
    public void updateInputs(ToFInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(
            distanceFromClaw,
            distanceStdDevClaw,
            isDetected,
            ambience,
            signalStrength,
            measurementHealth,
            measurementTime
        ).isOK();

        inputs.distanceMeters = distanceFromClaw.getValueAsDouble();
        inputs.distanceStdDevMeters = distanceStdDevClaw.getValueAsDouble();
        inputs.isDetected = isDetected.getValue();
        inputs.ambience = ambience.getValueAsDouble();
        inputs.signalStrength = signalStrength.getValueAsDouble();
        inputs.measurementHealth = measurementHealth.getValue().equals(MeasurementHealthValue.Good);
        inputs.measurementTime = measurementTime.getValueAsDouble();
    }
}