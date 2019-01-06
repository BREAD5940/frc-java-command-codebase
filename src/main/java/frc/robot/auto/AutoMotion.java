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

    /**
     * different heights of goals
     * LOW: the lowest level of the rocket; through the hatch of the cargo ship
     * MIDDLE: the middle level of the rocket
     * HIGH: the highest level of the rocket
     * OVER: dropped into the cargo ship from above
     */
    public enum goalHeight{
        LOW, MIDDLE, HIGH, OVER
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
        ArrayList<Command> toReturn = new ArrayList<Command>();
        if (gHeight == goalHeight.LOW){
            // Could also align with line
            // TODO find out the actual units for the speed
            toReturn.add(new FollowVisionTarget(1, 20));
        }else{
            //TODO Align with line (IR sensor?)
        }

        toReturn.add(new auto_Elevator(getElevatorHeight()));

        switch (sPiece){
            case HATCH:
                //TODO when we have a hatch outtake command, put it here
            case CARGO:
                //TODO when we have a cargo outtake command, put it here
            default:
                break;
        }

        return toReturn;

    }

    private double getElevatorHeight(){
        switch (this.gHeight){
            case LOW:
                switch (this.gType){
                    case CARGO:
                        return RobotConfig.auto.fieldPositions.cargo_ship_hatch;
                    case ROCKET:
                        switch (this.sPiece){
                            case CARGO:
                                return RobotConfig.auto.fieldPositions.low_rocket_port;
                            case HATCH:
                                return RobotConfig.auto.fieldPositions.low_rocket_hatch;
                        }
                }
            case MIDDLE:
                switch (this.sPiece){
                    case CARGO:
                        return RobotConfig.auto.fieldPositions.middle_rocket_port;
                    case HATCH:
                        return RobotConfig.auto.fieldPositions.middle_rocket_hatch;
                }
            case HIGH:
                switch (this.sPiece){
                    case CARGO:
                        return RobotConfig.auto.fieldPositions.high_rocket_port;
                    case HATCH:
                        return RobotConfig.auto.fieldPositions.high_rocket_hatch;
                }
            case OVER:
                return RobotConfig.auto.fieldPositions.cargo_ship_wall;
            default:
                return 0;
        }
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
