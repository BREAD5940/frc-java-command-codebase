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
import frc.robot.subsystems.drivetrain;

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


  public auto_action_DRIVE(double distance, String gear, double timeout) {
    this.targetDistance = distance;
    this.gear = gear;
    this.timeout = timeout;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
  }

  public static final drivetrain drivetrain  = new drivetrain();

  // Called just before this Command runs the first time
  @Override
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {


    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
