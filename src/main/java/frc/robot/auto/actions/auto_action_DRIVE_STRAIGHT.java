/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;



  /**
   * Drive straight ahead. The target angle is set based on the current robot angle.
   * This uses a position PID loop to set drivetrain speeds. The left and right speeds
   * are manipulated based on a turn PID loop such that the angle remains constant.
   * // TODO determine how the difference in encoder positions if the angle is changed will affect pid
   * @param distance in feet
   */
public class auto_action_DRIVE_STRAIGHT extends Command {
  double distance;
  String gear;
  double actionMaxSpeed;
  double timeout;
  double start_left_distance = Robot.drivetrain.m_left_talon.getSelectedSensorPosition(0);
  double start_right_distance = Robot.drivetrain.m_right_talon.getSelectedSensorPosition(0);
  double start_gyro_angle = Robot.gyro.getAngle();
  double target_gyro_angle = start_gyro_angle;
  double current_angle = Robot.gyro.getAngle();
  
  /**
   * Get the current angle error of the gyro
   */
  double getAngleError() { return (target_gyro_angle - Robot.gyro.getAngle()); }

  /**
   * auto_action_DRIVE_STRAIGHT drives in a straight line. The target angle is the same angle as the gyro
   * is initilized with.
   * @param distance in feet
   * @param speed maximum speed in feet per second
   * @param gear for the action
   * @param timeout after which the command will cleanly exit
   */
  public auto_action_DRIVE_STRAIGHT(double distance, double speed, String gear, double timeout) {
    this.distance = distance;
    this.gear = gear;
    this.actionMaxSpeed = speed;
    this.timeout = timeout;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Reset the gyro angles to the current angle. 
    // TODO do we want to do this, or allow setting an absolute angle, such that between auto actions the robot can correct for being knocked into?
    // (Currently, I'm thinking that it should be no...)
    start_gyro_angle = Robot.gyro.getAngle();

    setTimeout(timeout); // set the timeout

    // shift in case this has not been done yet
    if (gear == "low") { Robot.drivetrain.setLowGear(); }
    else if (gear == "high") { Robot.drivetrain.setHighGear(); }
    else { throw new IllegalArgumentException("Cannot set gear to " + this.gear + " !" ); }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    current_angle = Robot.gyro.getAngle();
    
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ( ((Math.abs(Robot.drivetrain.getRightDistance() - this.distance) < RobotConfig.drive_auto_position_tolerence) 
      // && (Math.abs(Robot.drivetrain.getLeftDistance() - this.distance) < RobotConfig.drive_auto_position_tolerence) 
      && (Math.abs(Robot.drivetrain.getLeftVelocity()) < RobotConfig.drive_auto_velocity_tolerence) 
      && (Math.abs(Robot.drivetrain.getRightVelocity()) < RobotConfig.drive_auto_position_tolerence)
      && (Math.abs(target_gyro_angle - current_angle) < RobotConfig.drive_auto_straight_angle_tolerence ))
      || (isTimedOut()) 
    ){ return true; }
    else { return false; }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
