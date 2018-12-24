/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Run the elevator to a set height during autonomous
 * @param demand in inches
 */
public class audo_Elevator extends Command {

  double demand;
  boolean isInstant;

  /**
   * Run the elevator to a set height during autonomous
   * @param demand in inches
   */
  public audo_Elevator(double demand, boolean instant) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    this.demand = demand;
    this.isInstant = instant;
  }

  // public static final drivetrain drivetrain  = new drivetrain();

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.setHeight(demand);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  /**
   * Return boolean of if the elevator is within 0.5 inches of the set height OR the command is supposed to run instantly. 
   */
  @Override
  protected boolean isFinished() {
    if ((isInstant) || (Math.abs(Robot.elevator.getHeight() - demand) < 0.5)) { 
      return true;
    }
    else {
      return false;
    }
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
