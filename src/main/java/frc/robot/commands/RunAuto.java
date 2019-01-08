/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
// import frc.robot.Robot;
import frc.robot.auto.AutoMotion.heldPiece;;

/**
 * Shifter command to shift to high gear
 */
public class RunAuto extends Command {
  public heldPiece piece;

  public RunAuto(heldPiece piece) {
    // Use requires() here to declare subsystem dependencies
    this.piece = piece;
    

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_auto = Robot.autoSelect.chooseMotion(piece);
    Robot.m_auto.getCommandGroup().start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // does nothing
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
