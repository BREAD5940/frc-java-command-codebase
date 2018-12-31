package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoPath {


    /**
     * the five different robot starting locations
     * 
     * CENTER: directly to the right of the exchange zone
     * LEFT: directly to the left of the exchange zone
     * RIGHT:
     * FAR_LEFT: 
     */
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
    AutoCommandGroup bigCommandGroup;

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
     *      list of sequential commandgroups for this path that are turned into one giant AutoCommandGroup
     */

    public AutoPath (String name, goals goal, robotLoc reqLocation, String setup, CommandGroup... commands){
        this.location  = reqLocation;
        this.setup = setup;
        this.name = name;
        this.goal = goal;
        this.bigCommandGroup = new AutoCommandGroup(commands);
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
        return this.bigCommandGroup;
    }
    
}