package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ToFIO {
    @AutoLog
    public static class ToFInputs {
        public boolean isConnected = false;
        public double distanceMeters = 0.0;
        public double distanceStdDevMeters = 0.0;
        public double ambience = 0.0;
        public double signalStrength = 0.0;
        public boolean measurementHealth = false;
        public boolean isDetected = false;
        public double measurementTime = 0.0;
    }

    public default void updateInputs(ToFInputs inputs) {
        inputs.isConnected = false;
        inputs.distanceMeters = 0.0;
        inputs.distanceStdDevMeters = 0.0;
        inputs.ambience = 0.0;
        inputs.signalStrength = 0.0;
        inputs.measurementHealth = false;
        inputs.isDetected = false;
        inputs.measurementTime = 0.0;
    };
}
