package frc.robot.subsystems.controls;

import java.util.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  public static enum AlgaePickupLevel {
    A1, A2;
  }

  public static enum AlgaeScoringLevel {
    PROCESSOR,
    NET
  }

  public static enum ReefFace {
    F1(Pipe.P01, Pipe.P12, AlgaePickupLevel.A1),
    F2(Pipe.P03, Pipe.P02, AlgaePickupLevel.A2),
    F3(Pipe.P05, Pipe.P04, AlgaePickupLevel.A1),
    F4(Pipe.P07, Pipe.P06, AlgaePickupLevel.A2),
    F5(Pipe.P09, Pipe.P08, AlgaePickupLevel.A1),
    F6(Pipe.P11, Pipe.P10, AlgaePickupLevel.A2);

    public final Pipe left;
    public final Pipe right;
    public final AlgaePickupLevel algaePickupLevel;

    private final Map<ReefFace, Integer> mBlueFaceToApriltag = new HashMap<>();
    private final Map<ReefFace, Integer> mRedFaceToApriltag = new HashMap<>();

    ReefFace(Pipe pLeft, Pipe pRight, AlgaePickupLevel pAlgaeLevel) {
        this.left = pLeft;
        this.right = pRight;
        this.algaePickupLevel = pAlgaeLevel;
    }

    public int getAprilTagTag() {
      return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Alliance.Red))
        ? mRedFaceToApriltag.get(this) : mBlueFaceToApriltag.get(this);
    }
  }

  private static final Map<Pipe, EnumSet<CoralLevel>> mScoredPoles = new EnumMap<>(Pipe.class);
  private static final Map<Integer, ReefFace> mApriltagToFace = new HashMap<>();

  private static final Map<ReefFace, Integer> mBlueFaceToApriltag = new HashMap<>();
  private  static final Map<ReefFace, Integer> mRedFaceToApriltag = new HashMap<>();

  static {
    mRedFaceToApriltag.put(ReefFace.F5, 6);
    mRedFaceToApriltag.put(ReefFace.F4, 7);
    mRedFaceToApriltag.put(ReefFace.F3, 8);
    mRedFaceToApriltag.put(ReefFace.F2, 9);
    mRedFaceToApriltag.put(ReefFace.F1, 10);
    mRedFaceToApriltag.put(ReefFace.F6, 11);

    // Blue side
    mBlueFaceToApriltag.put(ReefFace.F3, 17);
    mBlueFaceToApriltag.put(ReefFace.F4, 18);
    mBlueFaceToApriltag.put(ReefFace.F5, 19);
    mBlueFaceToApriltag.put(ReefFace.F6, 20);
    mBlueFaceToApriltag.put(ReefFace.F1, 21);
    mBlueFaceToApriltag.put(ReefFace.F2, 22);
  }

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

  public static int faceToTag(ReefFace face) {
    return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red))
      ? mRedFaceToApriltag.get(face) : mBlueFaceToApriltag.get(face);
  }

  public void setCurrentGamePiece(GamePiece pGamePiece) {
    mCurrentGamePiece = pGamePiece;
  }

  public InstantCommand setCurrentGamePieceCmd(GamePiece pGamePiece) {
    return new InstantCommand(()->mCurrentGamePiece = pGamePiece);
  }

  public void setCurrentCoralLevel(CoralLevel pCoralLevel) {
    mCurrentCoralLevel = pCoralLevel;
  }
  public CoralLevel getCurrentCoralLevel() {
    return mCurrentCoralLevel;
  }

  public GamePiece getCurrentGamePiece() {
    return mCurrentGamePiece;
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
