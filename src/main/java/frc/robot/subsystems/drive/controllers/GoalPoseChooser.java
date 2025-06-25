package frc.robot.subsystems.drive.controllers;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.controls.StateTracker;
import frc.robot.subsystems.controls.StateTracker.ReefFace;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.math.AllianceFlipUtil;

/* Chooses pose based of strategy and psoe */ 
public class GoalPoseChooser {
    public static enum CHOOSER_STRATEGY {
        kTest,
        kReefHexagonal,
        kCustom,
        kIntake,
        kNet
    }

    public static enum SIDE {
        LEFT, RIGHT, ALGAE
    }

    private static Pose2d customGoal = FieldConstants.AL;
    // @AutoLogOutput(key="GoalPoseChooser/Side")
    private static SIDE side = SIDE.RIGHT;
    private static ReefFace reefFace = ReefFace.F4;

    public static Pose2d getGoalPose(CHOOSER_STRATEGY strategy, Pose2d pose) {
        switch(strategy) {
            case kTest:
                return new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(60));
            case kReefHexagonal:
                return getReefHexagonalPose(pose);
            case kCustom:
                return customGoal;
            case kIntake:
                return getIntakePose(pose);
            case kNet:
                return getNetPose(pose);
        }
        return new Pose2d();
    }

    /* Splits the field into hexagon regions of the reef 
     * We got the left or right side of the side we are closest
     */
    public static Pose2d getReefHexagonalPose(Pose2d robotPose) {
        Rotation2d angleFromReefCenter = turnFromReefOriginForHexagon(robotPose);
        Pose2d goal;
        if(inBetween(-30.0, 30.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "D");
            reefFace = ReefFace.F1;

            if(side.equals(SIDE.LEFT)) {
                goal = getTargetPoseLeft(StateTracker.faceToTag(reefFace));
            }
            
            else if(side.equals(SIDE.RIGHT)) {
                goal = getTargetPoseRight(StateTracker.faceToTag(reefFace));
            }
            
            else goal = getTargetPose(StateTracker.faceToTag(reefFace));
        } 
        
        else if(inBetween(30.0, 90.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "E");
            reefFace = ReefFace.F2;
            if(side.equals(SIDE.LEFT)) {
                goal = getTargetPoseLeft(StateTracker.faceToTag(reefFace));
            }
            
            else if(side.equals(SIDE.RIGHT)) {
                goal = getTargetPoseRight(StateTracker.faceToTag(reefFace));
            }
            
            else goal = getTargetPose(StateTracker.faceToTag(reefFace));
        } 
        
        else if(inBetween(90.0, 150.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "F");
            reefFace = ReefFace.F3;
            if(side.equals(SIDE.LEFT)) {
                goal = getTargetPoseLeft(StateTracker.faceToTag(reefFace));
            }
            
            else if(side.equals(SIDE.RIGHT)) {
                goal = getTargetPoseRight(StateTracker.faceToTag(reefFace));
            }
            
            else goal = getTargetPose(StateTracker.faceToTag(reefFace));
        } 
        
        else if(inBetween(-150.0, -90.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "B");
            reefFace = ReefFace.F5;
            if(side.equals(SIDE.LEFT)) {
                goal = getTargetPoseLeft(StateTracker.faceToTag(reefFace));
            }
            
            else if(side.equals(SIDE.RIGHT)) {
                goal = getTargetPoseRight(StateTracker.faceToTag(reefFace));
            }
            
            else goal = getTargetPose(StateTracker.faceToTag(reefFace));
        } 
        
        else if(inBetween(-90.0, -30.0, angleFromReefCenter.getDegrees())){
            Logger.recordOutput("Drive/ReefSide", "C");
            reefFace = ReefFace.F6;
            if(side.equals(SIDE.LEFT)) {
                goal = getTargetPoseLeft(StateTracker.faceToTag(reefFace));
            }
            
            else if(side.equals(SIDE.RIGHT)) {
                goal = getTargetPoseRight(StateTracker.faceToTag(reefFace));
            }
            
            else goal = getTargetPose(StateTracker.faceToTag(reefFace));
        } 
        
        else {
            Logger.recordOutput("Drive/ReefSide", "A");
            reefFace = ReefFace.F4;
            if(side.equals(SIDE.LEFT)) {
                goal = getTargetPoseLeft(StateTracker.faceToTag(reefFace));
            }
            
            else if(side.equals(SIDE.RIGHT)) {
                goal = getTargetPoseRight(StateTracker.faceToTag(reefFace));
            }
            
            else goal = getTargetPose(StateTracker.faceToTag(reefFace));
        }
        // Logger.recordOutput("Drive/SelectedSide", side);

        return goal.plus(new Transform2d(0.0, 0.0, Rotation2d.fromDegrees(180)));
    }

    public static Pose2d getTargetPoseLeft(int pTagID) {
        return getTargetPose(pTagID, 0.04, DriverStation.isAutonomous() ?  PoseOffsets.AUTONLEFT.getOffsetM() : PoseOffsets.LEFT.getOffsetM());
    }

    public static Pose2d getTargetPoseRight(int pTagID) {
        return getTargetPose(pTagID, 0.04, DriverStation.isAutonomous() ?  PoseOffsets.AUTONRIGHT.getOffsetM() : PoseOffsets.RIGHT.getOffsetM());
    }

    public static Pose2d getTargetPose(int pTagID) {
        return getTargetPose(pTagID, 0.04, 0.0);
    }

    public static Pose2d getTargetPose(int pTagID, double pXOffsetM, double pYOffsetM) {
        Pose2d tagPose =
            Vision.k2025Field.getTagPose(pTagID).map(Pose3d::toPose2d).orElse(null);

        Translation2d tagTranslation =
            tagPose.getTranslation()
                .plus(
                    new Translation2d(
                        (DriveConstants.kRobotWidthXMeters
                            / 2.0), // Offset the robot length so the front is 0 meters away
                        0)
                    .rotateBy(tagPose.getRotation()));
            Rotation2d tagRotation =
                tagPose.getRotation().plus(new Rotation2d(Math.PI)); // Flip the robot from the tag

            // Move "pDistanceInches" in the direction the tag is facing
            Translation2d frontOfTag =
                tagTranslation.plus(new Translation2d(-pXOffsetM, pYOffsetM).rotateBy(tagRotation));

        Pose2d targetPose2d =
            new Pose2d(frontOfTag, tagRotation)
                .plus(
                    new Transform2d(
                        0,
                        0,
                        new Rotation2d(
                            Math.toRadians(0.0)))); // .plus(new Rotation2d(Math.toRadians(-90.0)));
        Logger.recordOutput("Robot/Vision/AprilTagSetpoint", targetPose2d);
        Logger.recordOutput("Robot/Vision/AltAprilTagSetpoint", AllianceFlipUtil.apply(targetPose2d));
        return targetPose2d;
    }

    public static void updateSideStuff() {
        Logger.recordOutput("Drive/SelectedSide", side);
    }

    public static Pose2d getIntakePose(Pose2d robotPose) {
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return AllianceFlipUtil.apply((robotPose.getY() < Constants.kFieldWidthMeters / 2.0) ? FieldConstants.B_IR : FieldConstants.B_IL);
        } else return (robotPose.getY() < Constants.kFieldWidthMeters / 2.0) ? FieldConstants.R_IL : FieldConstants.R_IR;
    }

    /* DO NOT USE X COORDINATE, REPLACE y holonomic speeds with driver controller when using this! */
    public static Pose2d getNetPose(Pose2d robotPose) {
        return AllianceFlipUtil.apply(new Pose2d(FieldConstants.kXNetLineMeters, 0.0, Rotation2d.k180deg));
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    public static Rotation2d turnFromReefOriginForHexagon(Pose2d robotPose) {
        Pose2d reefCenter = AllianceFlipUtil.apply(FieldConstants.kReefCenter);
        Rotation2d angleFromReefCenter = Rotation2d.fromRadians(
            Math.atan2(
                robotPose.getY() - reefCenter.getY(), 
                robotPose.getX() - reefCenter.getX()));

        Rotation2d finalAngle = angleFromReefCenter.times(-1.0);
        if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) finalAngle = angleFromReefCenter.plus(Rotation2d.k180deg).times(-1.0);
        Logger.recordOutput("Drive/GoalPoseAngle", finalAngle);
        return finalAngle;
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    public static Rotation2d turnFromReefOrigin(Pose2d robotPose) {
        Pose2d reefCenter = AllianceFlipUtil.apply(FieldConstants.kReefCenter);
        Rotation2d angleFromReefCenter = Rotation2d.fromRadians(
            Math.atan2(
                robotPose.getY() - reefCenter.getY(), 
                robotPose.getX() - reefCenter.getX()));
        if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) angleFromReefCenter = angleFromReefCenter.plus(Rotation2d.k180deg);
        Logger.recordOutput("Drive/GoalPoseAngle", angleFromReefCenter);
        return angleFromReefCenter;
    }

    /* Sets the goal using a command, meant to be used with buttonboard */
    public static Command setGoalCommand(Pose2d goalPose) {
        return Commands.runOnce(()-> customGoal = goalPose);
    }

    public static Command setSideCommand(SIDE reefSide) {
        return Commands.runOnce(() -> side = reefSide);
    } 

    public static void setSide(SIDE reefSide) {
        side = reefSide;
    } 

    private static boolean inBetween(double min, double max, double val) {
        return (val >= min) && (val < max);
    }

    @AutoLogOutput(key = "GoalPoseChooser/ReefFace")
    public static StateTracker.ReefFace getReefFace() {
        return reefFace;
    } 

    public static final double kDistBetweenBranchesCenter =
        Units.inchesToMeters(13); // MAKE THIS 13 BEFORE A MATCH
    public static final double kDistBetweenBranchesCenterWithAlgae = Units.inchesToMeters(13.5);
    public static final double kClawOffset = Units.inchesToMeters(0.5);

    public enum PoseOffsets {
        AUTONLEFT(kDistBetweenBranchesCenterWithAlgae / 2.0 + kClawOffset),
        AUTONRIGHT(-1 * kDistBetweenBranchesCenterWithAlgae / 2.0 - kClawOffset),
        LEFT(kDistBetweenBranchesCenter / 2.0 + kClawOffset),
        CENTER(0 + kClawOffset),
        RIGHT(-1 * kDistBetweenBranchesCenter / 2.0 - kClawOffset);

        public final double offset;

        private PoseOffsets(double offset) {
            this.offset = offset;
        }

        public double getOffsetM() {
            return this.offset;
        }
    };
}