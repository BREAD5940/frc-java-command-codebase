package frc.robot.lib.motion;

import frc.math.Pose2d;
import frc.math.Pose2dWithCurvature;
import frc.math.Rotation2d;
import jaci.pathfinder.Trajectory.Segment;

public class TrajectorySegment {
// dt, x, y, position, velocity, acceleration, jerk, heading, curvature
  public double velocity, acceleration, jerk, heading;

  Pose2dWithCurvature pose;

  public TrajectorySegment(Pose2dWithCurvature pose, double velocity, double acceleration,
    double jerk) {
      this.pose = pose;
      this.velocity = velocity;
      this.acceleration = acceleration;
      this.jerk = jerk;
      this.heading = heading;
    }
  
  public TrajectorySegment() {
    this(new Pose2dWithCurvature(), 0, 0, 0);
  }

  public static TrajectorySegment fromPathfinderSegment(double time, Segment seg) {
    Pose2dWithCurvature pose_ = new Pose2dWithCurvature(new Pose2d(seg.y, seg.x, Rotation2d.fromRadians(seg.heading)), 0); // TODO generate curvature
    return new TrajectorySegment(pose_, seg.velocity, seg.acceleration, seg.jerk);
  }

}