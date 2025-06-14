package frc.robot.subsystems.telemetry;

public class TelemetryConstants {

  public static final String frontRightCameraStream = "http://10.100.14.99:1184/stream.mjpg";
  public static final String frontLeftCameraStream = "http://10.100.14.99:1182/stream.mjpg";

  public enum Tabs {
    COMPETITION("Competition"),
    DRIVE("Drive"),
    ELEVATOR("Elevator"),
    INTAKE("Intake"),
    CLIMB("Climb"),
    VISION("Vision"),
    WRIST("Wrist");

    public final String tabName;

    private Tabs(String tabName) {
      this.tabName = tabName;
    }

    public String getName() {
      return this.tabName;
    }
  };
}
