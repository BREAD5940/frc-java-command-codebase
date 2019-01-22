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

  public TimedTrajectorySegment(TimedTrajectorySegment other) {
    this(other.time, other.pose(), other.velocity, other.acceleration, other.jerk);
  }

  public static TimedTrajectorySegment fromPathfinderSegment(double time, Segment seg) {
    Pose2dWithCurvature _pose_ = new Pose2dWithCurvature(new Pose2d(seg.y, seg.x, Rotation2d.fromRadians(seg.heading)), 0); // TODO generate curvature
    return new TimedTrajectorySegment(time, _pose_, seg.velocity, seg.acceleration, seg.jerk);
  }

  public Pose2dWithCurvature pose() {
    return pose_;
  }

}