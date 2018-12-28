package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutoPath {

    public enum robotLoc{
        CENTER, LEFT, RIGHT, FAR_LEFT, FAR_RIGHT
    }

    public enum goals{
        FAR_SWITCH, NEAR_SWITCH, SCALE, TEST
    }


    robotLoc location;
    String setup;
    String name;
    goals goal;
    AutoCommandGroup commandGroup;

    /**
     * creates a new AutoPath for the current match
     * @param name
     *      name of the path
     * @param goal
     *      goal of the autopath (FAR_SWITCH, NEAR_SWITCH, SCALE, TEST)
     * @param reqLocation
     *      required location of the robot (CENTER, LEFT, RIGHT, FAR_LEFT, FAR_RIGHT)
     * @param setup
     *      required field setup of scales and switches 
     * @param  commands
     *      list of sequential commands for this path, to be inputted into an auto_command_group
     */

    public AutoPath (String name, goals goal, robotLoc reqLocation, String setup, Command... commands){
        this.location  = reqLocation;
        this.setup = setup;
        this.name = name;
        this.goal = goal;
        this.commandGroup = new AutoCommandGroup(commands);
    }

    // id functions

    
    public String getName(){
        return this.name;
    }

    public goals getGoal(){
        return this.goal;
    }

    public robotLoc getReqLoc(){
        return this.location;
    }

    public String getReqSetup(){
        return this.setup;
    }

    public AutoCommandGroup getCommandGroup(){
        return this.commandGroup;
    }
    
}