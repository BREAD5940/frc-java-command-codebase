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
import frc.robot.lib.encoderlib;



/**
 * Auto action to drive a set distance.
 */
public class auto_action_DRIVE extends Command {
  double targetDistance;
  String gear;
  double targetSpeed;
  boolean isDone = false;
  double timeout;

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
    double targetSpeedRaw = encoderlib.distanceToRaw(targetSpeed, robotconfig.POSITION_PULSES_PER_ROTATION, robotconfig.left_wheel_effective_diameter) ;
    double startingDistanceLeft = Robot.drivetrain.getLeftDistance();
    double startingDistanceRight = Robot.drivetrain.getRightDistance();
    double endDistanceLeft = encoderlib.distanceToRaw(targetDistance, robotconfig.POSITION_PULSES_PER_ROTATION, robotconfig.left_wheel_effective_diameter);

    setTimeout(timeout); // set the timeout

    if (gear == "low") { Robot.drivetrain.setLowGear(); }
    else if (gear == "high") { Robot.drivetrain.setHighGear(); }
    else { throw new IllegalArgumentException("Cannot set gear to " + this.gear + " !" ); }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    

    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ( ((Math.abs(Robot.drivetrain.getRightDistance() - this.targetDistance) < robotconfig.drive_auto_position_tolerence) 
        && (Math.abs(Robot.drivetrain.getLeftDistance() - this.targetDistance) < robotconfig.drive_auto_position_tolerence) 
        && (Math.abs(Robot.drivetrain.getLeftVelocity()) < robotconfig.drive_auto_velocity_tolerence) 
        && (Math.abs(Robot.drivetrain.getRightVelocity()) < robotconfig.drive_auto_position_tolerence))
        || (isTimedOut()) ){
      return true;}
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
