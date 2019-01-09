package frc.robot.lib;

import edu.wpi.first.wpilibj.command.CommandGroup;
/**
 * This is an extention of CommandGroup that allows other classes to check if the commandgroup is finished
 * 
 * There's probably a less bad way to do this. Sorry.
 */
public abstract class AutoAction extends CommandGroup{
  public AutoAction(){
    super();
  }

  public boolean done(){
    return this.isFinished();
  }
}