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
    ArrayList<PurePoint> mOutput = new ArrayList<PurePoint>();
    double curvature, velocity;
    Point point;
    PurePoint toAppend;

    for(int i=0; i<input.length(); i++) {
      curvature = curvatureFromPathfinder(input, i);
      point = new Point(input.get(i).x, input.get(i).y);
      velocity = input.get(i).velocity;
      toAppend = new PurePoint(point, curvature, velocity);
      mOutput.add(toAppend);
    }


    return mOutput;
  }

    /**
      * To calculate curvature at a point we must first find the radius of 
      * the circle that intersects this point, the point ahead of it, and the point 
      * behind it. Once we have this, curvature is simply 1 / radius. From the pdf:
      * <p>
      * Given  P(x1, y1), Q(x2, y2), and R(x3, y3):
      * <p>
      * k  = 0.5 * ( x_1^2 + y_1^2 - Math.pow(x_2, 2) - y_^2)/(x_1 - x_2)
      * <p>
      * k_2 = (y_1 - y_2)/(x_1 - x_2)
      * <p> 
      * b = 0.5 * (Math.pow(x_2, 2) - 2 * x_2 * k_1 + Math.pow(y_2, 2) - Math.pow(x_3, 2) + 2 * x_3 * k_1 - Math.pow(y_3, 2)) / (x_3 * k_2 - y_3 + y_2 - x_2 * k_2)
      * <p>
      * a = k_1 - k_2 * b
      * <p>
      * r = sqrt((x_1 - a)^2 + (y_1 - b)^2)
      * <p>
      * curvature = 1/r
      * <p>
      * However, consider that if x_1 = x_2 you divide by zero. (try adding 0.0001 to x?)
      * Also, if 1/r = NaN, the curvature is zero.
      * Furthermore, the first and last points don't have any preceding points, so just
      * set their curvature to zero.
    */
  public double curvatureFromPathfinder(Trajectory path, int index) {

    double k_1, k_2, a, b, r;

    if ( (index == path.length()) || (index == 0) ) {
      return 0;
    }

    Point point1 = new Point(path.get(index-1).x,path.get(index-1).y);
    Point point2 = new Point(path.get(index).x,path.get(index).y);
    Point point3 = new Point(path.get(index+1).x,path.get(index+1).y);
    
    double x_1 = point1.x;
    double x_2 = point2.x;
    double x_3 = point3.x;

    double y_1 = point1.y;
    double y_2 = point2.y;
    double y_3 = point3.y;


    k_1  = 0.5 * ( Math.pow(x_1,2) + Math.pow(y_1,2) - Math.pow(x_2,2) - Math.pow(y_2,2))/(x_1 - x_2);
    k_2 = (y_1 - y_2)/(x_1 - x_2);
    b = 0.5 * (Math.pow(x_2, 2) - 2 * x_2 * k_1 + Math.pow(y_2, 2) - Math.pow(x_3, 2) + 2 * x_3 * k_1 - Math.pow(y_3, 2)) / (x_3 * k_2 - y_3 + y_2 - x_2 * k_2);
    a = k_1 - k_2 * b;
    r = Math.sqrt(Math.pow((x_1 - a),2) + Math.pow((y_1 - b),2));

    return 1/r;
  }

  public double curvature(ArrayList<PurePoint> mTraj) {
    return 0;
  }

  // public PurePoint getInitialOdometry() {
  //   Segment mInitialSegment = mSourceTraj.get(0);
  //   return new PurePoint(new Point(mInitialSegment.x, mInitialSegment.y), calculateCurvature(0), mInitialSegment.velocity);
  // }



}