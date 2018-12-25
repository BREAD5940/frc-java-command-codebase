// all of this is Wrong and False plz ignore

package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.auto.actions.auto_action_DRIVE;

public class auto_command_group extends CommandGroup{

    public auto_command_group(Command... commands) {
        for (Command command : commands){
            addSequential(command);
        }
    }
}