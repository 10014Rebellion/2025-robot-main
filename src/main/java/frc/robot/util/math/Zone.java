package frc.robot.util.math;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;

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
}