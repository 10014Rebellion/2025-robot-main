package frc.robot.util.math;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Zone {
    private ArrayList<Triangle> zones;

    public Zone(ArrayList<Triangle> zones) {
        this.zones = zones;
    }

    public boolean inZone(Pose2d pose) {
        for(Triangle zone : zones) {
            if(!zone.inTriangle(pose)) continue;
            return true;
        }
        return false;
    }

    public static Zone rectangularZone(Pose2d p1, Pose2d p2, Pose2d p3, Pose2d p4) {
        ArrayList<Triangle> rect = new ArrayList<Triangle>();

        rect.add(new Triangle(p1, p2, p3));
        rect.add(new Triangle(p2, p3, p4));

        return new Zone(rect);
    }

    public static Zone hexagnoalZone(Pose2d center,  double radius, Rotation2d startAngle) {
        ArrayList<Triangle> zones = new ArrayList<Triangle>();

        Rotation2d angle = startAngle;
        for(int i = 0; i < 5; i++) {
            zones.add(
                new Triangle(
                    center, 
                    center.plus(
                        new Transform2d(
                            radius * angle.getCos(), 
                            radius * angle.getSin(), 
                            new Rotation2d())), 
                    center.plus(
                        new Transform2d(
                            radius * angle.minus(Rotation2d.fromDegrees(60)).getCos(), 
                            radius * angle.minus(Rotation2d.fromDegrees(60)).getSin(), 
                            new Rotation2d()))));

            angle = angle.plus(Rotation2d.fromDegrees(60));
        }

        return new Zone(zones);
    }
}