package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.claw.ClawConstants.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class Autons {
  private static final Pose2d kStarting1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kStarting2 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kStarting3 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  private static final Pose2d kReef1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kReef2 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kReef3 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kReef4 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kReef5 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kReef6 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  private static final Pose2d kC1OrA1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kC2OrA2 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kC3OrA3 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  private static final Pose2d kNet = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kProcessor = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  private static final Pose2d kHPNorth = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private static final Pose2d kHPSouth = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  private static final Translation2d kReefCoralOffset = new Translation2d(0, 0);

  public Pose2d reefPoseToCoral(Pose2d pReefSide, boolean rightSide) {
    Translation2d offset = rightSide ? kReefCoralOffset : kReefCoralOffset.unaryMinus();
    return pReefSide.plus(new Transform2d(offset, new Rotation2d()));
  }

  public Autons(Drive pDrive, Elevator pElevator, Claw pClaw) {}
}
