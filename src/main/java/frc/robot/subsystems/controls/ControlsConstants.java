package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj.GenericHID;

public class ControlsConstants {
  public static class Buttonboard {
    public static final GenericHID buttonboard = new GenericHID(1);
    // Algae
    public static final int kAlgaePickupL2 = 9;
    public static final int kAlgaePickupL3 = 3;

    // Intake
    public static final int kReadyScoring = 4;
    public static final int kScoreCoral = 10;

    // Barge
    public static final int kEjectAlgaeToBarge = 5;
    public static final int kGoToBarge = 2;

    // Coral Scoring
    public static final int kSetScoreL4 = 6;
    public static final int kSetScoreL3 = 7;
    public static final int kSetScoreL2 = 1;
    public static final int kSetScoreL1 = 11;

    // Climbing
    public static final int kClimbLetGo = 12;
    public static final int kClimbPullUp = 8;
  }
}
