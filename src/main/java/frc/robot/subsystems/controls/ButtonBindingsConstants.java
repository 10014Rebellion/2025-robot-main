package frc.robot.subsystems.controls;


public class ButtonBindingsConstants {
  public static class DriverController {
    public static final int kDriverControllerPort = 0;
  }

  public static class Buttonboard {
    public static final int kButtonboardPort = 1;

    // Algae
    public static final int kAlgaePickupL2 = 9;
    public static final int kAlgaePickupL3 = 3;

    // Intake
    public static final int kPickup = 2;
    public static final int kScoreCoral = 10;

    // Barge
    public static final int kEjectAlgaeToBarge = 5;
    public static final int kGoToBarge = 4;

    // Coral Scoring
    public static final int kSetScoreL4 = 6;
    public static final int kSetScoreL3 = 7;
    public static final int kSetScoreL2 = 1;
    public static final int kSetScoreL1 = 11;

    // Climbing
    public static final int kClimbDescend = 8;
    public static final int kClimbAscend = 12;
  }
}
