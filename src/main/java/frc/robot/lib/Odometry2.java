package frc.robot.lib;

import frc.math.Pose2d;
import frc.math.Rotation2d;
import frc.math.Translation2d;
import frc.robot.Robot;

public class Odometry2 {
  // double x_, y_, theta_;
  double distance, deltaX, deltaY;
  Pose2d pose_;
  

  /**
   * The Odometry2 class provides methods for tracking the location of the robot
   * relative to the starting location of the robot plus a specified offset. If 
   * you set this to the "field center" coordinates odometry and Pure Persuit might^tm work
   * better, but it's likely fine.
   * <p>
   * his will set the robot pose to a x, y translation2d in feet and a theta in degrees.
   * <p>
   * @param x
   * @param y
   * @param theta
   */
  public Odometry2(double x, double y, double theta) {
    pose_ = new Pose2d(x, y, Rotation2d.fromDegrees(theta));
  }

  /**
   * Put the robot at a pose2d of 0, 0, 0
   */
  public Odometry2() {
    this(0, 0, 0);
  }

  /**
   * Incrament the trajectory by a given Translation2d
   * @param deltaMovement
   */
  public void incramentTrajectory(Translation2d deltaMovement) {
    pose_ = new Pose2d( new Translation2d(pose_.getTranslation(), deltaMovement) , pose_.getRotation() );
  }

  /**
   * Get the x dispalcement of this' pose_
   */
  public double getX() {
    return pose_.getTranslation().x();
  }

  /**
   * Get the y dispalcement of this' pose_
   */
  public double getY() {
    return pose_.getTranslation().y();
  }

  /**
   * Get the global theta of this' pose_ in degrees
   */
  public double getTheta() {
    return pose_.getRotation().getDegrees();
  }

}