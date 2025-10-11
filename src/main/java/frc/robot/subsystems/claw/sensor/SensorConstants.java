package frc.robot.subsystems.claw.sensor;

public class SensorConstants {
    public static final int CANRangeID = 55;
    public static final double coralDetectionCutoff = 0.04;
    public static final double kPositionTolerance = 3;
    public static final int kBeamBreakDIOPort = 2;

    public static ClawSensorsConfiguration clawSensorsConfiguration = 
        new ClawSensorsConfiguration(
            CANRangeID, 
            coralDetectionCutoff, 
            kPositionTolerance, 
            kBeamBreakDIOPort);


    public record ClawSensorsConfiguration(int CANRangeID, double coralDetectionCutoff, double positionTolerance, int beamBreakPort){}    
}
