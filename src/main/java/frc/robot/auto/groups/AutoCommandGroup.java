package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auto.AutoAction;

import java.util.ArrayList;

public class AutoCommandGroup extends AutoAction {

  ArrayList<Command> commands;

  // TODO change this to parallel?

  /**
   * just makes a proper command group out of the inputted Commands
   * @param commands
   *    an ArrayList of Commands
   */
  public AutoCommandGroup(ArrayList<Command> commands) {
    this.commands = commands;
    for (Command command : commands){
      addSequential(command);
    }
  }

}
