package frc.robot.lib.motion.purepursuit;

import frc.robot.lib.motion.Point;

/**
 * This object holds all the information needed by the Pure Pursuit
 * follower in order to follow a trajectory. It holds a Point, curvature,
 * velocity and acceleration.
 */
public class PurePoint {
  Point mPoint;
  
  double mCurvature;

  double mVelocity;

  /**
   * Create a new point at (0,0) with 0 curvature and zero velocity.
   */
  public PurePoint() {
    this.mPoint = new Point();
    this.mCurvature = 0;
    this.mVelocity = 0;
  }

  public PurePoint(Point point, double curvature, double velocity) {
    this.mPoint = point;
    this.mCurvature = curvature;
    this.mVelocity = velocity;
  }

}