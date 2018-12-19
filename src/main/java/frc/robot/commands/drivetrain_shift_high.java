/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.subsystems.drivetrain;



/**
 * Shifter command to shift to high gear
 */
public class drivetrain_shift_high extends Command {
  public drivetrain_shift_high() {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.drivetrain); // I don't think it's strictly necessary to require the drivetrain for an instant action
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.drivetrain.setHighGear();
    Robot.drivetrain_solenoid_shift_high();

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
