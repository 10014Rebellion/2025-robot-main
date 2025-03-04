package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class ControllerConstants {
  public static class Buttonboard {
    public static final GenericHID buttonboard = new GenericHID(1);
    // Barge
    public static final int kScoreBarge = 3;

    // Intake
    public static final int kSetLeftPose = 4;
    public static final int kSetRightPose = 10;

    // Claw Coral Grabbing
    public static final int kClawAimForPickup = 5;
    public static final int kClawGrabCoral = 2;

    // Coral Scoring
    public static final int kScoreL4 = 6;
    public static final int kScoreL3 = 7;
    public static final int kScoreL2 = 1;
    public static final int kScoreL1 = 11;

    // Climbing
    public static final int kClimbLetGo = 12;
    public static final int kClimbPullUp = 8;

    // Misc
    public static final int kLedMisguidance = 9;
  }
}
