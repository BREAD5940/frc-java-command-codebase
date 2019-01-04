package frc.robot.auto;

// import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutoPath {

    public enum robotLoc{
        CENTER, LEFT, RIGHT, FARL, FARR
    }

    public enum ssLoc{
        LLL, LLR, LRL, LRR, RRR, RLR, RLL, RRL
    }

    robotLoc location;
    ssLoc setup;
    String name;
    Command[] commands;

    /**
     * creates a new AutoPath for the current match
     * @param name
     *      name of the path
     * @param currentLocation
     *      current location of the robot (CENTER, LEFT, RIGHT, FARL, FARR)
     * @param setup
     *      required field setup of scales and switches (LLL, LLR, LRL, LRR, RRR, RLR, RLL, RRL)
     * @param  commands
     *      list of sequential commands for this path, to be inputted into an auto_command_group
     */

    public AutoPath (String name, robotLoc currentLocation, ssLoc setup, Command... commands){
        this.location  = currentLocation;
        this.setup = setup;
        this.name = name;
        this.commands = commands;
    }

    
}