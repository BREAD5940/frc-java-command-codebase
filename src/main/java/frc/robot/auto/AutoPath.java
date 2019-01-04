package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.auto.groups.*;

public class AutoPath {


    /**
     * the five different robot starting locations
     * 
     * CENTER: directly to the right of the exchange zone
     * LEFT: directly to the left of the exchange zone
     * RIGHT: the leftmost side of the right driver station
     * FAR_LEFT: the far left of the field
     * FAR_RIGHT: the far right of the field
     */
    public enum robotLoc{
        CENTER, LEFT, RIGHT, FAR_LEFT, FAR_RIGHT
    }

    public enum goals{
        FAR_SWITCH, NEAR_SWITCH, SCALE, TEST, LINE
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

    public AutoPath (String name, goals goal, robotLoc reqLocation, String setup, CommandGroup... commands){
        this.location  = reqLocation;
        this.setup = setup;
        this.name = name;
        this.goal = goal;
        this.bigCommandGroup = new AutoCommandGroup(genCommands());
    }

    public CommandGroup genCommands(){
        // TODO don't forget abt multi-cube plz
        switch (this.goal){
            case LINE:
                return new auto_PassLine(this.location);
            case SCALE:
                return new auto_Scale(this.location);
            case NEAR_SWITCH:
                return new auto_NearSwitch(this.location, getIndivSetup(goals.NEAR_SWITCH));
            case FAR_SWITCH:
                return new auto_FarSwitch(this.location);
            default:
                return new auto_TestDrive();
        }

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

    public char getIndivSetup(goals feature){
        switch(feature){
            case SCALE:
                return this.setup.charAt(1);
            case NEAR_SWITCH:
                return this.setup.charAt(0);
            case FAR_SWITCH:
                return this.setup.charAt(2);
            default:
                return 'x';
        }
    }

    public AutoCommandGroup getCommandGroup(){
        return this.bigCommandGroup;
    }
    
}