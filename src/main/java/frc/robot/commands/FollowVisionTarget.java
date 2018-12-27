/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.ShittyPID;
import edu.wpi.first.wpilibj.command.Command;


/**
 * Shifter command to shift to high gear
 */
public class FollowVisionTarget extends Command {
  double timeout;
  private ShittyPID forwardPID = new ShittyPID(
    RobotConfig.drive_straight.turn_kp, 
    RobotConfig.drive_auto_forward_velocity_min,
    RobotConfig.drive_auto_forward_velocity_max 
  );
  private ShittyPID turnPID = new ShittyPID(
    RobotConfig.drive_straight.turn_kp, 
    RobotConfig.drive_straight.turn_ki, 
    RobotConfig.drive_straight.minimum_turn_weight, 
    RobotConfig.drive_straight.maximum_turn_weight, 
    RobotConfig.drive_straight.turn_izone, 
    RobotConfig.drive_straight.turn_integral_max
  );
  

  public FollowVisionTarget(double speed, double timeout) {
    this.timeout = timeout;
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(timeout);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
