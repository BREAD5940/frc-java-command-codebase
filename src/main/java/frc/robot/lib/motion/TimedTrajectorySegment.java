package frc.robot.lib.motion;

import frc.math.Pose2d;
import frc.math.Pose2dWithCurvature;
import frc.math.Rotation2d;
import jaci.pathfinder.Trajectory.Segment;

/**
 * The timed trajectory segment is similar to the Segment class of Jaci's Pathfinder.
 * It holds a time, Pose2d with curvature, velocity, acceleration and jerk. Can be 
 * constructed using a TrajectorySegment and a time or explicitly with time, pose, velocity,
 * acceleration and jerk. 
 * 
 * @author Matthew Morley
 */
public class TimedTrajectorySegment {
// dt, x, y, position, velocity, acceleration, jerk, heading, curvature
  public double time, velocity, acceleration, jerk;

  private Pose2dWithCurvature pose_;

  public TimedTrajectorySegment(double time, Pose2dWithCurvature pose, double velocity, double acceleration,
    double jerk) {
      this.time = time;
      this.pose_ = pose;
      this.velocity = velocity;
      this.acceleration = acceleration;
      this.jerk = jerk;
    }
  public TimedTrajectorySegment(double time, TrajectorySegment seg) {
    this(time, seg.pose, seg.velocity, seg.acceleration, seg.jerk);
  }

  public TimedTrajectorySegment() {
    this(0, new TrajectorySegment());
  }

  public TimedTrajectorySegment(TimedTrajectorySegment other) {
    this(other.time, other.pose(), other.velocity, other.acceleration, other.jerk);
  }

  /**
   * Generate a TimedTrajectorySegment from a pathfinder
   * segment, the previous pathfinder segment and the
   * next pathfinder segment
   */
  public static TimedTrajectorySegment fromPathfinderSegment(double time, Segment cSegment, Segment lSegment, Segment nSegment) {

    // calcualte the curvature

    Pose2dWithCurvature _pose_ = new Pose2dWithCurvature(new Pose2d(cSegment.y, cSegment.x, Rotation2d.fromRadians(cSegment.heading)), 0); // TODO generate curvature
    return new TimedTrajectorySegment(time, _pose_, cSegment.velocity, cSegment.acceleration, cSegment.jerk);
  }

  /**
   * Calculate the curvature of the arc that passes through point 1 (last point), 2 (current point) 
   * and 3 (next point). From https://stackoverflow.com/questions/41144224/calculate-curvature-for-3-points-x-y
   * <p>
   * Positive curvature will be "rightward" and negative curvature is "leftward"
   */
  public static double curvatureFromPathfinder(Segment p0, Segment p1, Segment p2) {

    double dist1 = distanceFromPathfinder(p0, p1);
    double dist2 = distanceFromPathfinder(p1, p2);
    double dist3 = distanceFromPathfinder(p2, p0);

    double area = ((p0.x * (p1.y - p2.y)) 
            + (p1.x * (p2.y - p0.y)) 
            + (p2.x * (p0.y - p1.y))
        )/2.0f;

    double curvature = 4*area/(dist1*dist2*dist3);

    return curvature * -1; // TODO verify curvature sign
  }

  public static double distanceFromPathfinder(Segment p1, Segment p2) {
    double x = p2.x - p1.x;
    double y = p2.y - p1.y;
    return Math.sqrt(x * x + y * y);
  }

  public Pose2dWithCurvature pose() {
    return pose_;
  }

  @Override
  public String toString() {
    return "time: " + time + " pose: " + pose_.toString() + " velocity: " + velocity + " acceleration: " + acceleration + " jerk: " + jerk;
  }

}