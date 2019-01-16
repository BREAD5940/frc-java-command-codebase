package frc.robot.lib.motion.followers;

import java.util.Arrays;

import frc.robot.Robot;
import frc.robot.lib.motion.Odometer;
import frc.robot.lib.motion.purepursuit.DriveMotorState;
import frc.robot.lib.motion.purepursuit.PPWaypoint;
import frc.robot.lib.motion.purepursuit.Path;
import frc.robot.lib.motion.purepursuit.PathFollower;
import frc.robot.lib.motion.purepursuit.PathGenerator;
import frc.robot.lib.motion.purepursuit.Point;

public class PurePursuitFollower {
  Path path = null;
  PathFollower pathFollower = null;

  public void initPath() {
    // double angle = 0;
    Point pos = new Point();
    double maxVel = 2; // 2 feet/s
    double maxAcc = 1; // ft/sec every sec
    double spacing = 0.3, maxAngVel = 0.6; // 3 in simu
    double lookAheadDistance = 0.5/* 6 inches */;
    double trackWidth = 2;// 2 feet
    double targetTolerance = 1;// ft
    // double maxVel = 250, maxAcc = 70, spacing = 6, maxAngVel = 6;
    // double lookAheadDistance = 12;

    // double kLeftV = 1, kLeftA = 0, kLeftP = 0, kRightV = 1, kRightA = 0, kRightP
    // = 0, kSmooth = 0.8, kTolerance = 0.001;

    // generate path
    PathGenerator pathGenerator = new PathGenerator();
    // path = pathGenerator
    // .calculate(new Path(spacing, maxVel, maxAcc, maxAngVel, Arrays.asList(new
    // Waypoint(0, 0), new Waypoint(0, 2))));
    // path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
    // Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2), new Waypoint(-2, 2),
    // new Waypoint(-2, 6.5))));
    // path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
    // Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 4), new Waypoint(-2, 4),
    // new Waypoint(-2, 0))));
    path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
        Arrays.asList(new PPWaypoint(0, 0), new PPWaypoint(0, 4), new PPWaypoint(3, 7))));

    // log.log("Path: [" + path.waypoints.size() + "]");
    // log.log(path);
    // log.log("__________________________\n\n");
    // end generate path

    pathFollower = new PathFollower(path, lookAheadDistance, trackWidth, targetTolerance);
    pos = path.waypoints.get(0).p.copy();
    Odometer.getInstance().setX(pos.x);
    Odometer.getInstance().setY(pos.y);
  }

  public DriveMotorState getNextSignal() {
    return pathFollower.update(Odometer.getInstance().getPoint(), Robot.gyro.getAngle(), 1.0/50);
  }

  public boolean isDone() {
    return pathFollower.done;
  }
}