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
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConstants {

  public static final double kDistBetweenBranchesCenter = Units.inchesToMeters(13);
  public static final double kDistOffset = Units.inchesToMeters(1.5);

  public enum PoseOffsets {
    LEFT(kDistBetweenBranchesCenter / 2.0 + kDistOffset),
    CENTER(0),
    RIGHT(-1 * kDistBetweenBranchesCenter / 2.0 + kDistOffset);

    public final double offset;

    private PoseOffsets(double offset) {
      this.offset = offset;
    }

    public double getOffsetM() {
      return this.offset;
    }
  };

  public static double kScoringDistance = Units.inchesToMeters(0.0);

  public enum linearPoseOffsets {
    L4(Units.inchesToMeters(10)),
    L3(Units.inchesToMeters(4.5)),
    L2(Units.inchesToMeters(0.5)),
    L1(Units.inchesToMeters(0.25));

    public final double offset;

    private linearPoseOffsets(double offset) {
      this.offset = offset;
    }

    public double getOffsetM() {
      return this.offset;
    }
  };

  // Camera names (update if necessary)
  public static final String FRONT_LEFT_CAM = "FrontLeft-OV9281";
  public static final String FRONT_RIGHT_CAM = "FrontRight-OV9281";
  public static final String BACK_LEFT_CAM = "BackLeft-OV9281";
  public static final String BACK_RIGHT_CAM = "BackRight-OV9281";

  public static final double kRobotYLength = Units.inchesToMeters(35.5);
  public static final double kRobotXLength = Units.inchesToMeters(37.0);

  // Pose estimation strategies
  public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final PoseStrategy kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;

  // Vision measurement standard deviations
  public static final Matrix<N3, N1> kVisionSingleTagStandardDeviations = VecBuilder.fill(1, 1, 2);
  public static final Matrix<N3, N1> kVisionMultiTagStandardDeviations =
      VecBuilder.fill(0.5, 0.5, 1);

  // Max ambiguity for pose estimation
  public static final double kVisionMaxPoseAmbiguity = 0.2;

  private static final String CUSTOM_JSON_PATH = "apriltags/andymark/2025-no-barge.json";

  public static AprilTagFieldLayout kAprilTagFieldLayout =
      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  static {
    try {
      Path customPath = Path.of("/home/lvuser/deploy", CUSTOM_JSON_PATH);
      if (Files.exists(customPath)) {
        kAprilTagFieldLayout = new AprilTagFieldLayout(customPath);
      }
    } catch (IOException e) {
      DriverStation.reportError("Unable to load custom AprilTag JSON: " + e.getMessage(), true);
    }
  }

  /**
   * EXTREMELY IMPORTANT NOTE: The FrontLeft FrontRight, BackLeft, BackRight naming scheme is all
   * relative to the scoring side being the front of the bot (the side where the coral is scored on
   * the reef), BUT the values put into the Transform 3d is assuming the side perpendicular to the
   * elevator (battery side) is the front
   *
   * <p>Map of camera positions relative to the robot's center.
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
                      Units.inchesToMeters(12.908), // X: inches forward
                      Units.inchesToMeters(13.409), // Y: inches left
                      Units.inchesToMeters(12.77) // Z: inches above ground
                      ),
                  new Rotation3d(
                      Units.degreesToRadians(0), // Roll: No side tilt
                      Units.degreesToRadians(0), // Pitch: No upward tilt
                      Units.degreesToRadians(-30) // Yaw: (angled inward)
                      ))),

          // Front Right Camera (Mounted near FR swerve module)
          entry(
              FRONT_RIGHT_CAM,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(12.908), // X: inches forward
                      Units.inchesToMeters(-13.409), // Y: inches right
                      Units.inchesToMeters(12.77) // Z: inches above ground
                      ),
                  new Rotation3d(
                      Units.degreesToRadians(0), // Roll: No side tilt
                      Units.degreesToRadians(0), // Pitch: No upward tilt
                      Units.degreesToRadians(30) // Yaw: (angled inward)
                      ))),

          // Rear Left Camera (Mounted near BL swerve module)
          entry(
              BACK_LEFT_CAM,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-13.503), // X: inches back
                      Units.inchesToMeters(13.127), // Y: inches left
                      Units.inchesToMeters(12.6) // Z: inches off the ground
                      ),
                  new Rotation3d(
                      Units.degreesToRadians(0), // Roll: No side tilt
                      Units.degreesToRadians(0), // Pitch: No upwards tilt
                      Units.degreesToRadians(165) // Yaw: 15 degrees angled inwards
                      ))),

          // Rear Right Camera (Mounted near BR swerve module)
          entry(
              BACK_RIGHT_CAM,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-13.503), // X: inches forward
                      Units.inchesToMeters(-13.127), // Y: inches right
                      Units.inchesToMeters(12.6) // Z: inches off the ground
                      ),
                  new Rotation3d(
                      Units.degreesToRadians(0), // Roll: No side tilt
                      Units.degreesToRadians(0), // Pitch: No upwards tilt
                      Units.degreesToRadians(-165) // Yaw: 15 degrees angled inwards
                      ))));
}
