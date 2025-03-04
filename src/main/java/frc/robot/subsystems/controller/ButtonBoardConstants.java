package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBoardConstants {
  public static final GenericHID buttonboard = new GenericHID(1);

  // Unused button "5", lets bind this to an emote!

  // Barge
  public static final int kScoreBarge = 4;

  // Intake
  public static final int kIntakeDeploy = 9;
  public static final int kIntakeRetract = 3;

  // Claw Coral Grabbing
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
  public static final int kLEDMisguidance = 10;
}
