package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
/**
 * This is an extention of CommandGroup that allows other classes to check if the commandgroup is finished
 */
public abstract class AutoAction extends CommandGroup{
  public AutoAction(){
    super();
  }

  public boolean done(){
    return this.isFinished();
  }
}