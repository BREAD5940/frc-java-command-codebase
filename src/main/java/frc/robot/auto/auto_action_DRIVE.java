/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.robotconfig;
import frc.robot.lib.EncoderLib;



/**
 * Auto action to drive a set distance.
 */
public class auto_action_DRIVE extends Command {
  double targetDistance;
  String gear;
  double targetSpeed;
  boolean isDone = false;
  double timeout;
  double forward_kp;

  // things that change
  double forward_speed;
  double left_speed_raw;
  double right_speed_raw;

  /**
   * auto_action_DRIVE is a basic auto action. It should drive in a straight-ish line, as it uses 
   * nested PID loops to correct for errors caused by differing coefficients of friction. 
   * @param distance
   * @param gear
   * @param targetSpeed
   * @param timeout
   */
  public auto_action_DRIVE(double distance, String gear, double targetSpeed, double timeout) {
    this.targetDistance = distance;
    this.gear = gear;
    this.targetSpeed = targetSpeed;
    this.timeout = timeout;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double targetSpeedRaw = EncoderLib.distanceToRaw(targetSpeed, robotconfig.POSITION_PULSES_PER_ROTATION, robotconfig.left_wheel_effective_diameter) ;
    double startingDistanceLeft = Robot.drivetrain.getLeftDistance();
    double startingDistanceRight = Robot.drivetrain.getRightDistance();
    double endDistanceLeft = EncoderLib.distanceToRaw(targetDistance, robotconfig.POSITION_PULSES_PER_ROTATION, robotconfig.left_wheel_effective_diameter);

    setTimeout(timeout); // set the timeout

    // TODO set this kp based on each motor, or standardize it in robotconfig for both
    if (gear == "low") { Robot.drivetrain.setLowGear(); forward_kp = robotconfig.m_left_position_kp_low; }
    else if (gear == "high") { Robot.drivetrain.setHighGear(); forward_kp = robotconfig.m_left_position_kp_high; }
    else { throw new IllegalArgumentException("Cannot set gear to " + this.gear + " !" ); }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    forward_speed = Robot.drivetrain.shitty_P_loop(forward_kp, 
      targetDistance, 
      Robot.drivetrain.getLeftDistance(), 
      robotconfig.drive_auto_forward_velocity_min, 
      robotconfig.drive_auto_forward_velocity_max);
    double left_speed_raw = EncoderLib.distanceToRaw(forward_speed, robotconfig.left_wheel_effective_diameter / 12, robotconfig.POSITION_PULSES_PER_ROTATION) / 10;
    double right_speed_raw = EncoderLib.distanceToRaw(forward_speed, robotconfig.right_wheel_effective_diameter / 12, robotconfig.POSITION_PULSES_PER_ROTATION) / 10;

    Robot.drivetrain.setLeftSpeedRaw(600);//left_speed_raw);
    Robot.drivetrain.setRightSpeedRaw(600);//right_speed_raw);

    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if ( ((Math.abs(Robot.drivetrain.getRightDistance() - this.targetDistance) < robotconfig.drive_auto_position_tolerence) 
    //     && (Math.abs(Robot.drivetrain.getLeftDistance() - this.targetDistance) < robotconfig.drive_auto_position_tolerence) 
    //     && (Math.abs(Robot.drivetrain.getLeftVelocity()) < robotconfig.drive_auto_velocity_tolerence) 
    //     && (Math.abs(Robot.drivetrain.getRightVelocity()) < robotconfig.drive_auto_position_tolerence))
    //     || (isTimedOut()) ){
    //   return true;}
    // else { return false; }
    return false;
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
