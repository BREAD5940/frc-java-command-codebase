package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auto.actions.DriveStraight;
import frc.robot.auto.actions.DriveTrajectoryPathfinder;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.groups.PrepareIntake;
import frc.robot.subsystems.Elevator.ElevatorPresets;
import frc.robot.auto.actions.TurnInPlace;
import frc.robot.auto.groups.*;
import frc.robot.commands.FollowVisionTarget;
import frc.robot.RobotConfig;

import java.util.ArrayList;

import javax.lang.model.element.ElementKind;

public class AutoMotion {

  public enum heldPiece{
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
   * CARGO_CARGO: put cargo in the cargo ship
   * ROCKET_CARGO: put cargo in the rocket
   * CARGO_HATCH: put a hatch on the cargo ship
   * ROCKET_HATCH: put a hatch on the rocket
   * RETRIEVE_HATCH: pick up a hatch from the loading station
   */
  public enum goalType{
    CARGO_CARGO, CARGO_HATCH, ROCKET_CARGO, ROCKET_HATCH, RETRIEVE_HATCH
  }

  String setup;
  String name;
  goalHeight gHeight;
  goalType gType;
  heldPiece piece;
  AutoCommandGroup bigCommandGroup;

  /**
   * creates a new AutoPath for the current match
   * @param name
   *    name of the path
   * @param piece
   *    the type of game piece the robot is currently holding (HATCH, CARGO, NONE)
   * @param gHeight
   *    the height of the goal the robot should aim for (LOW, MIDDLE, HIGH)
   * @param gType
   *    the type of goal (ROCKET, CARGO)
   * @param  commands
   *    list of sequential commandgroups for this path that are turned into one giant AutoCommandGroup
   */

  public AutoMotion (String name, goalHeight gHeight, goalType gType){
    this.name = name;
    this.gHeight = gHeight;
    this.gType = gType;
    if (gType==goalType.CARGO_CARGO||gType==goalType.ROCKET_CARGO){
      this.piece = heldPiece.CARGO;
    }else if (gType==goalType.CARGO_HATCH||gType==goalType.ROCKET_HATCH){
      this.piece = heldPiece.HATCH;
    }else{
      this.piece=heldPiece.NONE;
    }
    if (piece!=heldPiece.NONE){
      this.bigCommandGroup = new AutoCommandGroup(genCommands());
    }else{
      this.bigCommandGroup = new AutoCommandGroup(new DriveTrajectoryPathfinder("yes"));
    }
  }


  /**
   * Generate commands based on the parameters of the current AutoMotion
   * @return
   *  returns an ArrayList of commands
   */
  private ArrayList<Command> genCommands(){
    ArrayList<Command> toReturn = new ArrayList<Command>();
    if (gHeight == goalHeight.LOW){
      // Could also align with line
      // TODO find out the actual units for the speed
      toReturn.add(new FollowVisionTarget(1, 20));
    }else{
      //TODO Align with line (IR sensor?)
    }

    toReturn.add(new PrepareIntake(getElevatorPreset()));

    switch (piece){
      case HATCH:
        toReturn.add(new PlaceHatch());
      case CARGO:
        if(gHeight == goalHeight.OVER){
          toReturn.add(new DropCargo(true));
        }else{
          toReturn.add(new DropCargo(false));
        }
      default:
        break;
    }

    return toReturn;

  }

  /**
   * selects the correct ElevatorPresets from RobotConfig based on the goalHeight, the goalType, and the heldPiece
   */
  private ElevatorPresets getElevatorPreset(){
    switch (gType){
      case CARGO_CARGO:
        if (gHeight == goalHeight.LOW){
          return ElevatorPresets.CARGO_SHIP_HATCH;
        }else{
          return ElevatorPresets.CARGO_SHIP_WALL;
        }
      case CARGO_HATCH:
        return ElevatorPresets.CARGO_SHIP_HATCH;
      case ROCKET_CARGO:
        if (gHeight == goalHeight.LOW){
          return ElevatorPresets.LOW_ROCKET_PORT;
        }else if (gHeight == goalHeight.MIDDLE){
          return ElevatorPresets.MIDDLE_ROCKET_PORT;
        }else{
          return ElevatorPresets.HIGH_ROCKET_PORT;
        }
      case ROCKET_HATCH:
        if (gHeight == goalHeight.LOW){
          return ElevatorPresets.LOW_ROCKET_HATCH;
        }else if (gHeight == goalHeight.MIDDLE){
          return ElevatorPresets.MIDDLE_ROCKET_HATCH;
        }else{
          return ElevatorPresets.HIGH_ROCKET_HATCH;
        }
      default:
        return ElevatorPresets.CARGO_SHIP_HATCH;
    }
  }

  // id functions

  /**
   * @return
   *  the name of the AutoMotion
   */
  public String getName(){
    return this.name;
  }

  /**
   * 
   * @return
   *  the required goalHeight of the AutoMotion
   */
  public goalHeight getGoalHeight(){
    return this.gHeight;
  }

  /**
   * 
   * @return
   *  the required goalType of the AutoMotion
   */
  public goalType getGoalType(){
    return this.gType;
  }

  /**
   * 
   * @return
   *  the required heldPiece of the AutoMotion
   */
  public heldPiece getheldPiece(){
    return this.piece;
  }

  /**
   * 
   * @return
   *  the full AutoCommandGroup of the AutoMotion
   */
  public AutoCommandGroup getCommandGroup(){
    return this.bigCommandGroup;
  }
  
}
