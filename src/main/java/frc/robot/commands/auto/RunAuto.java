/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.AutoCombo.CurrentLocation;
import frc.robot.commands.auto.AutoCombo.GoalLocation;
import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;


/**
 * Selects and runs an auto command group
 */
public class RunAuto extends Command {

  public GoalType gt;
  public GoalLocation goal;
  public GoalHeight height;
  public AutoMotion motion;
  public AutoCombo cMotion;
  public CurrentLocation location;
  public boolean isDrive;


  public RunAuto(GoalType gt, GoalHeight height) {
    // Use requires() here to declare subsystem dependencies
    this.gt = gt;
    this.height = height;
    this.isDrive = false;
  }

  public RunAuto(GoalType gt, GoalHeight height,CurrentLocation location,GoalLocation goal){
    this(gt, height);
    this.location = location;
    this.goal = goal;
    this.isDrive = true;
  }

  @Override
  protected void initialize() {
    if(!isDrive){
      motion = new AutoMotion(height, gt);
      motion.getBigCommandGroup().start();
    }else{
      cMotion = new AutoCombo(height, gt, location,goal);
      cMotion.getBigCommandGroup().start();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Don't need to do anything here
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return motion.getBigCommandGroup().done();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() { }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() { }
}
