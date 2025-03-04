package frc.robot.subsystems.vision;

import static java.util.Map.entry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.Map;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConstants {

  public static final double kDistBetweenBranchesCenter = 0.33;

  public enum PoseOffsets {
    LEFT(kDistBetweenBranchesCenter / 2.0),
    CENTER(0),
    RIGHT(-kDistBetweenBranchesCenter / 2.0);

    public final double offset;

    private PoseOffsets(double offset) {
      this.offset = offset;
    }

    public double getOffsetM() {
      return this.offset;
    }
  };

  public static final int[] redReefTagIDs = {17, 18, 19, 20, 21, 22};
  public static final int redNorthCoralTagID = 2;
  public static final int redSouthCoralTagID = 1;
  public static final int redSideProcessor = 3;

  public static final int[] blueReefTagIDs = {6, 7, 8, 9, 10, 11};
  public static final int blueNorthCoralTagID = 13;
  public static final int blueSouthCoralTagID = 12;
  public static final int blueSideProcessor = 16;

  // Camera names (update if necessary)
  public static final String FRONT_LEFT_CAM = "FrontLeft-OV9281";
  public static final String FRONT_RIGHT_CAM = "FrontRight-OV9281";
  public static final String BACK_LEFT_CAM = "BackLeft-OV9281";
  public static final String BACK_RIGHT_CAM = "BackRight-OV9281";

  public static final double kRobotYLength = Units.inchesToMeters(37.0);

  // Pose estimation strategies
  public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final PoseStrategy kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;

  // Vision measurement standard deviations
  public static final Matrix<N3, N1> kVisionSingleTagStandardDeviations = VecBuilder.fill(1, 1, 2);
  public static final Matrix<N3, N1> kVisionMultiTagStandardDeviations =
      VecBuilder.fill(0.5, 0.5, 1);

  // Max ambiguity for pose estimation
  public static final double kVisionMaxPoseAmbiguity = 0.2;

  // Load the official 2025 AprilTag field layout
  public static final AprilTagFieldLayout kAprilTagFieldLayout =
      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  /**
   * Map of camera positions relative to the robot's center.
   *
   * <p>- **Translation3d (X, Y, Z)**: - X: Forward (+) / Backward (-) relative to the center of the
   * bot - Y: Left (+) / Right (-) relative to the center of the bot - Z: Up (+) / Down (-) relative
   * to the ground, most likely wont be inside the ground
   *
   * <p>- **Rotation3d (Roll, Pitch, Yaw)**: - Roll (X-axis rotation): Side tilt (it will prolly be
   * 0 unless we do some crazy stuff) - Pitch (Y-axis rotation): Camera looking up/down (Negative =
   * up, yeah ik its weird, blame WPILIB not me) - Yaw (Z-axis rotation): Camera turning left/right.
   * Imagine a birds eye view of the bot, 0deg is north, 90 is west, -90 is east, and 180 is south
   */
  public static final Map<String, Transform3d> cameraPositions =
      Map.ofEntries(

          // Front Left Camera (Mounted near FL swerve module)
          entry(
              FRONT_LEFT_CAM,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(14.416), // X: inches forward
                      Units.inchesToMeters(10.576), // Y: inches left
                      Units.inchesToMeters(9.144) // Z: inches above ground
                      ),
                  new Rotation3d(
                      Units.degreesToRadians(0), // Roll: No side tilt
                      Units.degreesToRadians(0), // Pitch: No upward tilt
                      Units.degreesToRadians(-44) // Yaw: (angled inward)
                      ))),

          // Front Right Camera (Mounted near FR swerve module)
          entry(
              FRONT_RIGHT_CAM,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(14.416), // X: inches forward
                      Units.inchesToMeters(-10.576), // Y: inches right (negative)
                      Units.inchesToMeters(9.144) // Z: inches above ground
                      ),
                  new Rotation3d(
                      Units.degreesToRadians(0), // Roll: No side tilt
                      Units.degreesToRadians(0), // Pitch: No upward tilt
                      Units.degreesToRadians(45) // Yaw: (angled inward)
                      ))),

          // Rear Left Camera (Mounted near BL swerve module, positions TBD)
          entry(
              BACK_LEFT_CAM,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(0), // X: TBD
                      Units.inchesToMeters(0), // Y: TBD
                      Units.inchesToMeters(0) // Z: TBD
                      ),
                  new Rotation3d(
                      Units.degreesToRadians(0), // Roll: No side tilt
                      Units.degreesToRadians(0), // Pitch: No upwards tilt
                      Units.degreesToRadians(0) // Yaw: TBD
                      ))),

          // Rear Right Camera (Mounted near BR swerve module, positions TBD)
          entry(
              BACK_RIGHT_CAM,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(0), // X: TBD
                      Units.inchesToMeters(0), // Y: TBD
                      Units.inchesToMeters(0) // Z: TBD
                      ),
                  new Rotation3d(
                      Units.degreesToRadians(0), // Roll: No side tilt
                      Units.degreesToRadians(0), // Pitch: No upwards tilt
                      Units.degreesToRadians(0) // Yaw: TBD
                      ))));
}
