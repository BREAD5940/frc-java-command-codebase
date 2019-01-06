package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;

public class AutoCommandGroup extends CommandGroup{

    // TODO change this to parallel?
    /**
     * just makes a proper command group out of the inputted command groups
     * @param commands
     */
    public AutoCommandGroup(CommandGroup... commands) {
        for (CommandGroup command : commands){
            addSequential(command);
        }
    }

    /**
     * just makes a proper command group out of the inputted command groups
     * @param commands
     */
    public AutoCommandGroup(Command... commands) {
        for (Command command : commands){
            addSequential(command);
        }
    }
}