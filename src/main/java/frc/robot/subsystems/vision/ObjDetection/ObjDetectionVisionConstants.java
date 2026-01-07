package frc.robot.subsystems.vision.ObjDetection;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.AprilTagDetection.AprilTagVisionConstants.Orientation;

public class ObjDetectionVisionConstants {
    // From CAD and decided by you in configuration

    public record ObjectOrientation(
        double area,
        double pitch,
        double yaw,
        boolean inverted){}

    // CURRENTLY THE LEFT CAMERA WE SHALL REPLACE //
    public static final String kTopCamName = "Gold3_OV9281";
    public static final Orientation kTopCamOrientnation = Orientation.FRONT;
    public static final Transform3d kTopCamTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.284), // X: inches forward
            Units.inchesToMeters(12.7829), // Y: inches left
            Units.inchesToMeters(12.769) // Z: inches above ground
        ),
        new Rotation3d(
            Units.degreesToRadians(0), // Roll: No side tilt
            Units.degreesToRadians(0), // Pitch: No upward tilt
            Units.degreesToRadians(-30) // Yaw: (angled inward)
        )
    );

}

