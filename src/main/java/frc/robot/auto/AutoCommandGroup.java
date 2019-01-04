package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

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
}