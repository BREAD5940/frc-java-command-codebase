package frc.robot.lib.motion.followers;

import java.io.File;
import java.util.ArrayList;

import frc.robot.lib.motion.Point;

import frc.robot.lib.motion.purepursuit.PurePoint;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * This class takes a Pathfinder spline and 
 * follows it using a Pure Pursuit follower.
 */
public class PurePursuitFollower {
  Trajectory mSourceTraj;

  /**
   * Create a pure pursuit follower given a Pathfinder
   * source trajectory
   */
  public PurePursuitFollower(Trajectory traj) {
    mSourceTraj = traj;
  }

  /**
   * Create a pure pursuit follower given a filepath
   * to a source trajectory
   */
  public PurePursuitFollower(String filepath) {
    File traj = new File("/home/lvuser/deploy/paths/test.pf1.csv");
    mSourceTraj = Pathfinder.readFromCSV(traj);
  }

  public ArrayList<PurePoint> convert(Trajectory input) {
    ArrayList<PurePoint> mOutput;
    double curvature, velocity;
    Point point;
    PurePoint toAppend;

    for(int i=0; i<input.length(); i++) {
      curvature = curvatureFromPathfinder(i);
      point = new Point(input.get(i).x, input.get(i).y)
      velocity = input.get(i).velocity;
      toAppend = new PurePoint(point, curvature, velocity);
      mOutput.add(toAppend);
    }


    return null;
  }

  public double curvatureFromPathfinder(int index) {
    return 0;
  }

  public double curvature(ArrayList<PurePoint> mTraj) {
    return 0;
  }

  // public PurePoint getInitialOdometry() {
  //   Segment mInitialSegment = mSourceTraj.get(0);
  //   return new PurePoint(new Point(mInitialSegment.x, mInitialSegment.y), calculateCurvature(0), mInitialSegment.velocity);
  }



}