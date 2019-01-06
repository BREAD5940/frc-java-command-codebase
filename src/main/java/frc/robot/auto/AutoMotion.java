package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auto.actions.auto_DriveStraight;
import frc.robot.auto.groups.*;

public class AutoMotion {

    public enum startingPiece{
        HATCH, CARGO, NONE
    }

    public enum goalHeight{
        LOW, MIDDLE, HIGH
    }

    // TODO remember there's actually a difference for cargo, but not for hatches
    /**
     * different types of goals on the field
     * CARGO: the cargo ship
     * ROCKET: the rocket
     */
    public enum goalType{
        CARGO, ROCKET
    }

    String setup;
    String name;
    goalHeight gHeight;
    goalType gType;
    startingPiece sPiece;
    AutoCommandGroup bigCommandGroup;

    /**
     * creates a new AutoPath for the current match
     * @param name
     *      name of the path
     * @param startingPiece
     *      the type of game piece the robot is currently holding (HATCH, CARGO, NONE)
     * @param gHeight
     *      the height of the goal the robot should aim for (LOW, MIDDLE, HIGH)
     * @param gType
     *      the type of goal (ROCKET, CARGO)
     * @param  commands
     *      list of sequential commandgroups for this path that are turned into one giant AutoCommandGroup
     */

    public AutoMotion (String name, startingPiece sPiece, goalHeight gHeight, goalType gType){
        this.name = name;
        this.gHeight = gHeight;
        this.gType = gType;
        this.sPiece = sPiece;
        this.bigCommandGroup = new AutoCommandGroup(genCommands());
    }

    public CommandGroup genCommands(){
        // TODO align with auto tape/line; select up/amount up; check type of game piece; place piece

        if (gHeight == goalHeight.LOW){
            //Align with tape OR line

        }else{
            //Align with line

        }

        return new AutoCommandGroup(new auto_DriveStraight(2));

    }

    // id functions

    public String getName(){
        return this.name;
    }

    public goalHeight getGoalHeight(){
        return this.gHeight;
    }

    public goalType getGoalType(){
        return this.gType;
    }

    public startingPiece getStartingPiece(){
        return this.sPiece;
    }

    public AutoCommandGroup getCommandGroup(){
        return this.bigCommandGroup;
    }
    
}