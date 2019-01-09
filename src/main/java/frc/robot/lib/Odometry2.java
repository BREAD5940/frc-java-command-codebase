package frc.robot.lib;

import frc.robot.Robot;

public class Odometry2 {
  double x_, y_, theta_;
  double distance, deltaX, deltaY;
  

  /**
   * The Odometry2 class provides methods for tracking the location of the robot
   * relative to the starting location of the robot plus a specified offset. If 
   * you set this to the "field center" coordinates odometry and Pure Persuit might^tm work
   * better, but it's likely fine.
   */
  public Odometry2(double x, double y, double theta) {
    this.x_ = x;
    this.y_ = y;
    this.theta_ = theta;
  }

  /**
   * The Odometry2 class provides methods for tracking the location of the robot
   * relative to the starting location of the robot.
   */
  public Odometry2() {
    this.x_ = 0;
    this.y_ = 0;
    this.theta_ = 0;
  }
  

  public void update(double leftDelta, double rightDelta, double globalTheta) {
    distance = (leftDelta + rightDelta) / 2;
    deltaX = distance * cosine(globalTheta);
    deltaY = distance * sine(globalTheta);
  }

  /**
   * Simple method for calculating the cosine of an
   * angle in degrees.
   * @param theta in degrees
   * @return the cosine of that angle
   */
  public double cosine(double theta) {
    return Math.cos(Math.toRadians(theta));
  }

  /**
   * Simple method for calculating the sine of an
   * angle in degrees.
   * @param theta in degrees
   * @return the sin of that angle
   */
  public double sine(double theta) {
    return Math.sin(Math.toRadians(theta));
  }


}