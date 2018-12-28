package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.auto.actions.auto_action_DRIVE;

public class AutoCommandGroup extends CommandGroup{

    /**
     * literally just makes a proper sequential command group out of the inputted commands
     * @param commands
     */
    public AutoCommandGroup(Command... commands) {
        for (Command command : commands){
            addSequential(command);
        }
    }
}