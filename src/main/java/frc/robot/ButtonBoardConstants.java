package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.vision.VisionConstants.PoseOffsets;

public class ButtonBoardConstants {
  public static final GenericHID buttonboard = new GenericHID(1);

  // Unused button "5", lets bind this to an emote!

  // Barge
  public static final int kScoreBarge = 4;

  // Which Side
  public static final int kRightSide = 9;
  public static final int kLeftSide = 3;

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

  // States
  public static heightPositions currentLevel;
  public static PoseOffsets currentSide;

  public enum heightPositions {
    L1(ElevatorConstants.Positions.L1, ClawConstants.Wrist.Positions.L1, 10.0),
    L2(ElevatorConstants.Positions.L2, ClawConstants.Wrist.Positions.L2, 10.0),
    L3(ElevatorConstants.Positions.L3, ClawConstants.Wrist.Positions.L3, 10.0),
    L4(ElevatorConstants.Positions.L4, ClawConstants.Wrist.Positions.L4, 10.0);

    public final ElevatorConstants.Positions elevatorPosition;
    public final ClawConstants.Wrist.Positions clawPosition;
    public final double offsetDistance;

    private heightPositions(ElevatorConstants.Positions pElevatorPos, 
    ClawConstants.Wrist.Positions pClawPos, double pOffset) {
      this.elevatorPosition = pElevatorPos;
      this.clawPosition = pClawPos;
      this.offsetDistance = pOffset;
    }

    public ElevatorConstants.Positions getElevatorPos() {
      return this.elevatorPosition;
    }

    public ClawConstants.Wrist.Positions getClawPos() {
      return this.clawPosition;
    }

    public double getOffset() {
      return this.offsetDistance;
    }
  };
}
