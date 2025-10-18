package frc.robot.subsystems.intake.BeamBreak;

public class BeamBreakConstants {
    public static int kFrontSensorDIOPort = 3;
    public static int kBackSensorDIOPort = 4;

    public static boolean kFrontSensorInvert = true;
    public static boolean kBackSensorInvert = true;

    public static BeamBreakConfig frontHardware = new BeamBreakConfig(kFrontSensorDIOPort, kFrontSensorInvert);
    public static BeamBreakConfig backHardware = new BeamBreakConfig(kBackSensorDIOPort, kBackSensorInvert);


    public record BeamBreakConfig(
        int kID,
        boolean kInvert
    ){}
}
