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
 * Run the elevator to a set height during autonomous using one of the constructors lol.
 * When I wrote this I just realized you could have multiple constructors, so you could 
 * have a bunch of different parameters, so I went crazy with mixtures of demand, isInstant,
 * and timeout.
 */
public class auto_Elevator extends Command {

  double demand;
  boolean isInstant;
  double timeout = 10; // default dimeout to 10s

  /**
   * Run the elevator to a set height during autonomous. Also set boolean flag
   * for if this command runs instantly or not. Timeout defaults to 10 seconds.
   * @param demand in inches
   * @param isInstant flag
   */
  public auto_Elevator(double demand, boolean isInstant) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    this.demand = demand;
    this.isInstant = isInstant;
  }

  /**
   * This comstructor defaults to waiting for the elevator to reach the target height
   * set in inches. Timeout will default to 10 seconds 
   */
  public auto_Elevator(double demand) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    this.demand = demand;
    this.isInstant = false;
  }

  /**
   * Run the elevator to a set height during autonomous. Also set boolean flag
   * for if this command runs instantly or not. Timeout will *not* default and 
   * must be set in arguments.
   * @param demand in inches
   * @param isInstant flag
   * @param timeout of this command in seconds
   */
  public auto_Elevator(double demand, boolean isInstant, double timeout) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    this.demand = demand;
    this.isInstant = isInstant;
    this.timeout = timeout;
  }

  /**
   * This comstructor defaults to waiting for the elevator to reach the target height
   * set in inches. Timeout will *not* default and must be set in arguments.
   * @param demand in inches
   * @param timeout of this command in seconds
   */
  public auto_Elevator(double demand, double timeout) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    this.demand = demand;
    this.isInstant = false;
    this.timeout = timeout;
  }

  @Override
  protected void initialize() {
    Robot.elevator.setHeight(demand);
    setTimeout(timeout);
  }

  @Override
  protected void execute() {
  }

  /**
   * Return boolean of if the elevator is within 0.5 inches of the set height OR the command is supposed to run instantly. 
   */
  @Override
  protected boolean isFinished() {
    if ((isInstant) || (Math.abs(Robot.elevator.getHeight() - demand) < 0.5) || isTimedOut() ) { 
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
