package frc.robot.lib;

import frc.math.Pose2d;
import frc.math.Rotation2d;

public class PursuitWaypoint {

  // protected final double x_;
  // protected final double y_;
  // protected final double theta_;
  protected final Pose2d pose_;
  protected final double curvature_;
  protected final double velocity_;

  public PursuitWaypoint(double x, double y, double theta, double curvature, double velocity) {
    pose_ = new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    curvature_ = curvature;
    velocity_ = velocity;
  }

}