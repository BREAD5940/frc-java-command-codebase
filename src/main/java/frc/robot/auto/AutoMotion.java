package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.auto.actions.auto_DriveStraight;
import frc.robot.auto.actions.auto_Elevator;
import frc.robot.auto.groups.*;
import frc.robot.commands.FollowVisionTarget;
import frc.robot.RobotConfig;

import java.util.ArrayList;

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
        if (sPiece!=startingPiece.NONE){
            this.bigCommandGroup = new AutoCommandGroup(genCommands());
        }else{
            System.out.println("No starting piece. Aborting auto section");
        }
    }

    private ArrayList<Command> genCommands(){
        // TODO Things this should do: align with auto tape/line; raise 'elevator'; place piece
        ArrayList<Command> toReturn = new ArrayList<Command>();
        if (gHeight == goalHeight.LOW){
            //Align with tape OR line
            // TODO find out the actual units for the speed
            toReturn.add(new FollowVisionTarget(1, 20));
        }else{
            //TODO Align with line
            //Raise elevator
            //there's got to be a less-bad way to do this
            double elevatorHeight = 0;
            switch (gHeight){
                case MIDDLE:
                    switch (sPiece){
                        case CARGO:
                            elevatorHeight = RobotConfig.auto.fieldPositions.middle_rocket_port;
                        case HATCH:
                            elevatorHeight = RobotConfig.auto.fieldPositions.middle_rocket_hatch;
                    }
                case HIGH:
                    switch (sPiece){
                        case CARGO:
                            elevatorHeight = RobotConfig.auto.fieldPositions.high_rocket_port;
                        case HATCH:
                            elevatorHeight = RobotConfig.auto.fieldPositions.high_rocket_hatch;
                    }
            }

            toReturn.add(new auto_Elevator(elevatorHeight));
        }

        switch (sPiece){
            case HATCH:
                //TODO Place hatch pannel
            case CARGO:
                //TODO Place cargo
            default:
                break;
        }

        return toReturn;

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
