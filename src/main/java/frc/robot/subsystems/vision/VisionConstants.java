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
  public static String cam1Name = "";
  public static String cam2Name = "";
  public static String cam3Name = "";
  public static String cam4Name = "";

  public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final PoseStrategy kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
  public static final Matrix<N3, N1> kVisionSingleTagStandardDeviations = VecBuilder.fill(1, 1, 2);
  public static final Matrix<N3, N1> kVisionMultiTagStandardDeviations =
      VecBuilder.fill(0.5, 0.5, 1);
  public static final double kVisionMaxPoseAmbiguity = 0.2;

  public static final AprilTagFieldLayout kAprilTagFieldLayout =
      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  public static final Map<String, Transform3d> cameraPositions =
      Map.ofEntries(
          // Note: idk which way is which, very possible everything is "as expected" aside from
          // Pitch
          // Pitch is inverted so up is negative
          // Also camera positions are very much up in the air right now hence the zeros (2/9/2025)
          entry(
              "FrontLeft",
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(15.5),
                      Units.inchesToMeters(12),
                      Units.inchesToMeters(11.5)),
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(-40),
                      Units.degreesToRadians(-30)))),
          entry(
              "FrontRight",
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(15.5),
                      Units.inchesToMeters(-12),
                      Units.inchesToMeters(11.5)),
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(-40),
                      Units.degreesToRadians(30)))),
          entry(
              "RearLeft",
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0)))),
          entry(
              "RearRight",
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0)))));
}
