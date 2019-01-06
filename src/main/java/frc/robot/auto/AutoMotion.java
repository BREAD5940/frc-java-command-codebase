package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.auto.groups.*;

public class AutoMotion {

    public enum startingPiece{
        HATCH, CARGO, NONE
    }

    public enum goals{
        LOW, MIDDLE, HIGH
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
     *      goal of the autopath (FAR_SWITCH, NEAR_SWITCH, SCALE, TEST, LINE)
     * @param reqLocation
     *      required location of the robot (CENTER, LEFT, RIGHT, FAR_LEFT, FAR_RIGHT)
     * @param setup
     *      required field setup of scales and switches 
     * @param  commands
     *      list of sequential commandgroups for this path that are turned into one giant AutoCommandGroup
     */

    public AutoMotion (String name, startingPiece sPiece, goals goal, CommandGroup... commands){
        this.name = name;
        this.goal = goal;
        this.bigCommandGroup = new AutoCommandGroup(genCommands());
    }

    public CommandGroup genCommands(){
        // TODO align with auto tape/line; select up/amount up; check type of game piece; place piece

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


    public AutoCommandGroup getCommandGroup(){
        return this.bigCommandGroup;
    }
    
}