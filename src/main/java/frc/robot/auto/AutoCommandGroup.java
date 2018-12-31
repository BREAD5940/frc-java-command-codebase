package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCommandGroup extends CommandGroup{

    /**
     * literally just makes a proper sequential command group out of the inputted command
     * @param commands
     */
    public AutoCommandGroup(CommandGroup... commands) {
        for (CommandGroup command : commands){
            addSequential(command);
        }
    }
}