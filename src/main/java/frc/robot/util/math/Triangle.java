package frc.robot.util.math;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Triangle {
    private Pose2d checkedPose = new Pose2d();

    private final BooleanSupplier side3;
    private final BooleanSupplier side2;
    private final BooleanSupplier side1;

    public Triangle(Pose2d pose1, Pose2d pose2, Pose2d pose3) {
        double m3 = (pose2.getY() - pose1.getY()) / (pose2.getX() - pose1.getX());
        double m2 = (pose3.getY() - pose1.getY()) / (pose3.getX() - pose1.getX());
        double m1 = (pose3.getY() - pose2.getY()) / (pose3.getX() - pose2.getX());

        double b3 = -1 * m3 * pose1.getX() + pose1.getY();
        double b2 = -1 * m2 * pose1.getX() + pose1.getY();
        double b1 = -1 * m1 * pose2.getX() + pose2.getY();

        int infinityCount = 0;

        if(!Double.isInfinite(m3)) {
            if(pose3.getY() < m3 * pose3.getX() + b3) side3 = () -> checkedPose.getY() < m3 * checkedPose.getX() + b3;
            else if(pose3.getY() > m3 * pose3.getX() + b3) side3 = () -> checkedPose.getY() > m3 * checkedPose.getX() + b3;
            else {
                side3 = null;
                DriverStation.reportError("DO IT!!!", true);
            }
        } else {
            if(infinityCount > 0) {
                side3 = null;
                DriverStation.reportError("DO IT!!!", true);
            }
            else if(pose3.getX() < pose1.getX()) side3 = () -> checkedPose.getX() < pose1.getX();
            else if(pose3.getX() > pose1.getX()) side3 = () -> checkedPose.getX() > pose1.getX();
            else {
                side3 = null;
                DriverStation.reportError("DO IT!!!", true);
            }
        }

        if(!Double.isInfinite(m2)) {
            if(pose2.getY() < m2 * pose2.getX() + b2) side2 = () -> checkedPose.getY() < m2 * checkedPose.getX() + b2;
            else if(pose2.getY() > m2 * pose2.getX() + b2) side2 = () -> checkedPose.getY() > m2 * checkedPose.getX() + b2;
            else {
                side2 = null;
                DriverStation.reportError("DO IT!!!", true);
            }
        } else {
            if(infinityCount > 0) {
                side2 = null;
                DriverStation.reportError("DO IT!!!", true);
            }
            else if(pose2.getX() < pose1.getX()) side2 = () -> checkedPose.getX() < pose1.getX();
            else if(pose2.getX() > pose1.getX()) side2 = () -> checkedPose.getX() > pose1.getX();
            else {
                side2 = null;
                DriverStation.reportError("DO IT!!!", true);
            }
        }

        if(!Double.isInfinite(m1)) {
            if(pose1.getY() < m1 * pose1.getX() + b1) side1 = () -> checkedPose.getY() < m1 * checkedPose.getX() + b1;
            else if(pose1.getY() > m1 * pose1.getX() + b1) side1 = () -> checkedPose.getY() > m1 * checkedPose.getX() + b1;
            else {
                side1 = null;
                DriverStation.reportError(
                    "Y:" + pose1.getY() + "\n m:" + m1 + "\nX:" + pose1.getX() + "\nb:" + b1, true);
            }
        } else {
            if(infinityCount > 0) {
                side1 = null;
                DriverStation.reportError("DO IT!!!", true);
            }
            else if(pose1.getX() < pose3.getX()) side1 = () -> checkedPose.getX() < pose3.getX();
            else if(pose1.getX() > pose3.getX()) side1 = () -> checkedPose.getX() > pose3.getX();
            else {
                side1 = null;
                DriverStation.reportError("DO IT!!!", true);
            }
        }
    }

    public boolean inTriangle(Pose2d checkedPose) {
        this.checkedPose = checkedPose;
        return side3.getAsBoolean() && side2.getAsBoolean() && side1.getAsBoolean();
    }
}
