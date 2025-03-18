package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true; // CHANGE THIS IF YOU WANT TO ENABLE TUNING PID


  public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final double kFieldLengthMeters = kFieldLayout.getFieldLength();
  public static final double kFieldWidthMeters = kFieldLayout.getFieldWidth();

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ControllerConstants {
    // Coral Buttons
    public static int kL1Button = 0; // TO DO: Configure me!
    public static int kL2Button = 1; // TO DO: Configure me!
    public static int kL3Button = 2; // TO DO: Configure me!
    public static int kL4Button = 3; // TO DO: Configure me!
    public static int kIntakeCoralButton = 9; // TO DO: Configure me!

    // Algae Buttons
    public static int kBargeButton = 4; // TO DO: Configure me!
    public static int kProcessorButton = 5; // TO DO: Configure me!

    // Climb Buttons
    public static int kClimbUp = 6; // TO DO: Configure me!
    public static int kClimbDown = 7; // TO DO: Configure me!
    public static int kAutoClimb = 8; // TO DO: Configure me!
  }
}
