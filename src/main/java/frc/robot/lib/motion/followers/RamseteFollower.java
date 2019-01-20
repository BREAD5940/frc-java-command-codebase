package frc.robot.lib.motion.followers;

import java.io.File;

import frc.math.Pose2d;
import frc.math.Rotation2d;
import frc.robot.lib.motion.DriveMotorState;
import frc.robot.lib.motion.Odometer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;


/**
 * This class handles all the stuff necessary to calculate angular and linear velocities for a ramsete follower.
 * Thanks to Pantherbotics at https://github.com/Pantherbotics/FRC-2018-PowerUp/
 */
public class RamseteFollower {
  

  private static final double b = 1.0;                // greater than zero; increases correction
  private static final double zeta = 0.2;             // between zero and one; increases dampening
  private double wheelBase;
  private int segmentIndex;
  private Trajectory path;                            //this is the path that we will follow
  private Odometer odo;                               //this is the robot's x and y position, as well as its heading

  public RamseteFollower(double wheelBase, Trajectory path) {
      System.out.println("Initializing Ramsete Follower");
      this.wheelBase = wheelBase;
      this.path = path;
      segmentIndex = 0;
      this.odo = Odometer.getInstance(); // set up Odometer object
  }

  public RamseteFollower(double wheelBase, String sourceTrajName) {
    this(wheelBase, Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/" + sourceTrajName + ".pf1.csv")));
  }

  private double calcW_d() {
      if (segmentIndex < path.length()-1) {
          double lastTheta = path.get(segmentIndex).heading;
          double nextTheta = path.get(segmentIndex + 1).heading;
          return (nextTheta - lastTheta) / path.get(segmentIndex).dt;
      } else
          return  0;
  }

  public DriveMotorState getNextDriveSignal() {
      double left = 0;
      double right = 0;
      if (isFinished()) {
          return new DriveMotorState(left, right, true);
      }
      System.out.println("Getting segment segmentIndex number: " + segmentIndex + " out of " + (path.length()-1) + " segments");

      Segment current = path.get(segmentIndex);                                                   //look at segment of path
      double w_d = calcW_d();                                                                     //need to find wanted rate of change of heading

      double v = calcVel(current.x, current.y, current.heading, current.velocity, w_d);           //v = linear velocity
      double w = calcAngleVel(current.x, current.y, current.heading, current.velocity, w_d);      //w = angular velocity

      //v = clamp(v, -20, 20);                                                           //clamp values to be between -20 and 20 fps
      //w = clamp(w, Math.PI * -2.0, Math.PI * 2.0);                                     //clamp values to be between -2pi and 2pi rad/s

      System.out.println("Velocity " + v + " Angular Velocity " + w);

      left = (-wheelBase * w) / 2 + v;                                                            //do math to convert angular velocity + linear velocity
      right = (+wheelBase * w) / 2 + v;                                                           //into left and right wheel speeds (fps)


      System.out.println("Left: " + left + " Right: " + right);
      segmentIndex++;
      return new DriveMotorState(left, right);
  }

  public void setOdometry(Odometer odometer) {
      odo = odometer;
  }

  private double calcVel(double x_d, double y_d, double theta_d, double v_d, double w_d) {
      double k = calcK(v_d, w_d);
      double thetaError = theta_d-odo.getTheta();
      thetaError = Pathfinder.d2r(Pathfinder.boundHalfDegrees(Pathfinder.r2d(thetaError)));
      return v_d * Math.cos(thetaError) + k * (Math.cos(odo.getTheta()) * (x_d - odo.getX()) + Math.sin(odo.getTheta()) * (y_d - odo.getY()));
  }

  private double calcAngleVel(double x_d, double y_d, double theta_d, double v_d, double w_d) {
      double k = calcK(v_d, w_d);
      System.out.println("Theta" + odo.getTheta());
      double thetaError = theta_d - odo.getTheta();
      thetaError = Pathfinder.d2r(Pathfinder.boundHalfDegrees(Pathfinder.r2d(thetaError)));
      double sinThetaErrOverThetaErr;
      if (Math.abs(thetaError) < 0.00001)
          sinThetaErrOverThetaErr = 1; //this is the limit as sin(x)/x approaches zero
      else
          sinThetaErrOverThetaErr = Math.sin(thetaError) / (thetaError);
      return w_d + b * v_d * (sinThetaErrOverThetaErr) * (Math.cos(odo.getTheta()) * (y_d - odo.getY()) - Math.sin(odo.getTheta()) * (x_d - odo.getX())) + k * (thetaError); //from eq. 5.12
  }

  private double calcK(double v_d, double w_d) {
      return 2 * zeta * Math.sqrt(Math.pow(w_d, 2) + b * Math.pow(v_d, 2)); //from eq. 5.12
  }

  private double clamp(double value, double min, double max){
      if(value > max){
          return max;
      } else if(value < min){
          return min;
      }
      else
          return value;
  }

  public Pose2d getInitialPose() {
      return new Pose2d(path.get(0).x,path.get(0).y, Rotation2d.fromRadians(path.get(0).heading));
  }

  public boolean isFinished() {
      return segmentIndex == path.length();
  }

}