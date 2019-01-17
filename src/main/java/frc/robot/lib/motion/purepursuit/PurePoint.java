package frc.robot.lib.motion.purepursuit;

import java.text.DecimalFormat;

import frc.robot.lib.motion.purepursuit.Point;

/**
 * This object holds all the information needed by the Pure Pursuit
 * follower in order to follow a trajectory. It holds a Point, curvature,
 * velocity and acceleration.
 */
public class PurePoint {
  Point mPoint;

  double mVelocity;

  /**
   * Create a new point at (0,0) with 0 curvature and zero velocity.
   */
  public PurePoint() {
    this.mPoint = new Point();
    this.mVelocity = 0;
  }

  public PurePoint(Point point, double velocity) {
    this.mPoint = point;
    this.mVelocity = velocity;
  }


	public PurePoint subtract(PurePoint waypoint) {
		return new PurePoint(this.mPoint.subtract(waypoint.mPoint), this.mVelocity - waypoint.mVelocity);
	}

	public PurePoint add(PurePoint waypoint) {
		return new PurePoint(this.mPoint.add(waypoint.mPoint), this.mVelocity + waypoint.mVelocity);
	}

	public PurePoint scale(double scale) {
		return new PurePoint(this.mPoint.scale(scale), this.mVelocity * scale);
	}

	public PurePoint copy() {
		return new PurePoint(mPoint.copy(), this.mVelocity);
	}



  @Override
  public String toString(){
    final DecimalFormat fmt = new DecimalFormat("#0.000");
    return "(" + mPoint.toString() + "," + "," + "Velocity: " + fmt.format(mVelocity);
  }

}