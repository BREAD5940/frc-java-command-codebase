/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.AutoMotion.mGoalHeight;
import frc.robot.commands.auto.AutoMotion.mGoalType;


/**
 * Selects and runs an auto command group
 */
public class RunAuto extends Command {
  public mGoalType goal;
  public mGoalHeight height;
  public AutoMotion motion;

  public RunAuto(mGoalType goal, mGoalHeight height) {
    // Use requires() here to declare subsystem dependencies
    this.goal = goal;
    this.height = height;


  }

  @Override
  protected void initialize() {
    motion = new AutoMotion(height, goal);
    motion.mBigCommandGroup.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Scheduler.getInstance().run();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return motion.mBigCommandGroup.done();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() { }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() { }
}
