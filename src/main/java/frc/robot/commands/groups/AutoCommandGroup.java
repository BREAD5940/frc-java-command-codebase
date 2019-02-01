package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;

import java.util.ArrayList;

public class AutoCommandGroup extends CommandGroup {

  ArrayList<Command> commands;

  // TODO change this to parallel?

  /**
   * just makes a proper command group out of the inputted Commands. 
   * These commands are added sequentially to the group
   * @param commands
   *    an ArrayList of Commands
   */
  public AutoCommandGroup(ArrayList<Command> commands) {
    this.commands = commands;
    for (Command command : commands){
      addSequential(command);
    }
  }

  public boolean done(){
    return this.isFinished();
  }

}
