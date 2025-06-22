package frc.robot.subsystems.controls;

import java.util.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateTracker extends SubsystemBase {
  public static enum GamePiece {
    Coral,
    Algae
  }

  public static enum CoralLevel {
    T, B1, B2, B3;
  }

  public static enum Pipe {
    P01, P02, P03, P04, P05, P06, P07, P08, P09, P10, P11, P12;
  }

  public static enum AlgaeLevel {
    BR, A1, A2, N;
  }

  public static enum ReefFace {
    F1(Pipe.P01, Pipe.P12, AlgaeLevel.A1),
    F2(Pipe.P03, Pipe.P02, AlgaeLevel.A2),
    F3(Pipe.P05, Pipe.P04, AlgaeLevel.A1),
    F4(Pipe.P07, Pipe.P06, AlgaeLevel.A2),
    F5(Pipe.P09, Pipe.P08, AlgaeLevel.A1),
    F6(Pipe.P11, Pipe.P10, AlgaeLevel.A2);

    public final Pipe left;
    public final Pipe right;
    public final AlgaeLevel algaeLevel;

    ReefFace(Pipe pLeft, Pipe pRight, AlgaeLevel pAlgaeLevel) {
        this.left = pLeft;
        this.right = pRight;
        this.algaeLevel = pAlgaeLevel;
    }
  }

  private static final Map<Pipe, EnumSet<CoralLevel>> mScoredPoles = new EnumMap<>(Pipe.class);
  private static final Map<Integer, ReefFace> mApriltagToFace = new HashMap<>();

  private static CoralLevel mCurrentCoralLevel = CoralLevel.B3;
  private static GamePiece mCurrentGamePiece = GamePiece.Coral; 

  public StateTracker() {
    for (Pipe Pipe : Pipe.values()) {
      mScoredPoles.put(Pipe, EnumSet.noneOf(CoralLevel.class));
    }

    // Red side
    mApriltagToFace.put(6, ReefFace.F5);
    mApriltagToFace.put(7, ReefFace.F4);
    mApriltagToFace.put(8, ReefFace.F3);
    mApriltagToFace.put(9, ReefFace.F2);
    mApriltagToFace.put(10, ReefFace.F1);
    mApriltagToFace.put(11, ReefFace.F6);

    // Blue side
    mApriltagToFace.put(17, ReefFace.F3);
    mApriltagToFace.put(18, ReefFace.F4);
    mApriltagToFace.put(19, ReefFace.F5);
    mApriltagToFace.put(20, ReefFace.F6);
    mApriltagToFace.put(21, ReefFace.F1);
    mApriltagToFace.put(22, ReefFace.F2);
  }

  public void setCurrentGamePiece(GamePiece pGamePiece) {
    mCurrentGamePiece = pGamePiece;
  }

  public void setCurrentCoralLevel(CoralLevel pCoralLevel) {
    mCurrentCoralLevel = pCoralLevel;
  }

  public CoralLevel getCurrentCoralLevel() {
    return mCurrentCoralLevel;
  }

  public ReefFace getFaceFromTag(int pTagID) {
    return mApriltagToFace.get(pTagID);
  }

  public void markScored(Pipe pPipe, CoralLevel pCoralLevel) {
    mScoredPoles.get(pPipe).add(pCoralLevel);
  }

  public boolean isScored(Pipe pPipe, CoralLevel pCoralLevel) {
    return mScoredPoles.get(pPipe).contains(pCoralLevel);
  }

  public boolean isPipeFullyScored(Pipe pipe) {
    return mScoredPoles.get(pipe).containsAll(EnumSet.allOf(CoralLevel.class));
  }

  public boolean isFaceFullyScored(ReefFace face) {
    return isPipeFullyScored(face.left) && isPipeFullyScored(face.right);
  }

  public Pipe getLeftPipe(ReefFace pFace) {
    return pFace.left;
  }

  public Pipe getRightPipe(ReefFace pFace) {
    return pFace.right;
  }

  @Override
  public void periodic() {

  }
}
