package frc.robot.subsystems.controls;

import java.util.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateTracker extends SubsystemBase {
  public static enum Branch {
    B1, B2, B3;
  }

  public static enum Pipe {
    P01, P02, P03, P04, P05, P06, P07, P08, P09, P10, P11, P12;
  }

  public static enum AlgaeLevel {
    A1, A2;
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

  private static final Map<Pipe, EnumSet<Branch>> mScoredPoles = new EnumMap<>(Pipe.class);
  private static final Map<Integer, ReefFace> mApriltagToFace = new HashMap<>();


  public StateTracker() {
    for (Pipe Pipe : Pipe.values()) {
      mScoredPoles.put(Pipe, EnumSet.noneOf(Branch.class));
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

  public ReefFace getFaceFromTag(int pTagID) {
    return mApriltagToFace.get(pTagID);
  }

  public void markScored(Pipe pPipe, Branch pBranch) {
    mScoredPoles.get(pPipe).add(pBranch);
  }

  public boolean isScored(Pipe pPipe, Branch pBranch) {
    return mScoredPoles.get(pPipe).contains(pBranch);
  }

  public boolean isPipeFullyScored(Pipe pipe) {
    return mScoredPoles.get(pipe).containsAll(EnumSet.allOf(Branch.class));
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
