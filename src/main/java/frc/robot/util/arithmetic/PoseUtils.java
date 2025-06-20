package frc.robot.util.arithmetic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseUtils {

  /**
   * Shifts a given pose by a specified distance forward and right relative to the
   * pose's current
   * orientation.
   *
   * @param originalPose  The original pose to shift.
   * @param forwardMeters The distance to shift forward, relative to the pose's
   *                      heading.
   * @param rightMeters   The distance to shift right, relative to the pose's
   *                      heading.
   * @return A new Pose2d representing the shifted position.
   */
  public static Pose2d shiftPose(Pose2d originalPose, double forwardMeters, double rightMeters) {
    // Create a translation relative to the current orientation
    Translation2d translation = new Translation2d(forwardMeters, rightMeters).rotateBy(originalPose.getRotation());

    // Create a transform using the rotated translation
    Transform2d transform = new Transform2d(translation, new Rotation2d());

    // Apply the transformation to the original pose
    return originalPose.transformBy(transform);
  }

  /**
   * Rotates a given pose by a specified rotation and then shifts it by a
   * specified distance forward
   * and right relative to the rotated pose's new orientation.
   *
   * @param originalPose  The original pose to rotate and shift.
   * @param rotationDelta The amount of rotation to apply to the pose before
   *                      shifting.
   * @param forwardMeters The distance to shift forward, relative to the rotated
   *                      pose's heading.
   * @param rightMeters   The distance to shift right, relative to the rotated
   *                      pose's heading.
   * @return A new Pose2d representing the rotated and shifted position.
   */
  public static Pose2d shiftPose(
      Pose2d originalPose, Rotation2d rotationDelta, double forwardMeters, double rightMeters) {
    // Rotate the original pose by the specified rotationDelta
    Pose2d rotatedPose = new Pose2d(originalPose.getTranslation(), originalPose.getRotation().plus(rotationDelta));

    // Create a translation relative to the newly rotated orientation
    Translation2d translation = new Translation2d(forwardMeters, rightMeters).rotateBy(rotatedPose.getRotation());

    // Create a transform using the rotated translation
    Transform2d transform = new Transform2d(translation, new Rotation2d());

    // Apply the transformation to the rotated pose
    return rotatedPose.transformBy(transform);
  }
}
