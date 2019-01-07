package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;

import java.util.ArrayList;

public class AutoCommandGroup extends CommandGroup{

  // TODO change this to parallel?
  /**
   * just makes a proper command group out of the inputted CommandGroups
   * @param commands
   *    a list of CommandGroups
   */
  public AutoCommandGroup(CommandGroup... commands) {
    for (CommandGroup command : commands){
      addSequential(command);
    }
  }

  public AutoCommandGroup(Command... commands) {
    for (Command command : commands){
      addSequential(command);
    }
  }

  /**
   * just makes a proper command group out of the inputted Commands
   * @param commands
   *    an ArrayList of Commands
   */
  public AutoCommandGroup(ArrayList<Command> commands) {
    for (Command command : commands){
      addSequential(command);
    }
  }
}
