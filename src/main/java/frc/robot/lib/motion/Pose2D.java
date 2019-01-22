package frc.robot.lib.motion;

import jaci.pathfinder.Pathfinder;

public class Pose2D {
    public double x, y, theta;

    public Pose2D() {
        x = 0;
        y = 0;
        theta = 0;
    }

    public Pose2D(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose2D copy() {
        return new Pose2D(x, y, theta);
    }

    @Override
    public String toString() {
        return String.format("X: % 8f\tY: % 8f\tΘ: % 8f degrees", x, y, Pathfinder.boundHalfDegrees(Pathfinder.r2d(theta)));
    }
}