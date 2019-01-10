package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auto.actions.DriveTrajectoryPathfinder;
import frc.robot.commands.groups.PrepareIntake;
import frc.robot.subsystems.Elevator.ElevatorPresets;
import frc.robot.auto.groups.*;
import frc.robot.commands.FollowVisionTarget;
import frc.robot.commands.SetElevatorHeight;

import java.util.ArrayList;

import javax.lang.model.element.ElementKind;

/**
 * Creates a command group for a specific automatic motion.
 * Input a type of goal and a height then start the bigCommandGroup externally
 * 
 * @author Jocelyn McHugo
 */
public class AutoMotion {

  public enum heldPiece{
    HATCH, CARGO, NONE
  }
  
  /**
   * different heights of goals.
   * LOW: the lowest level of the rocket & through the hatch of the cargo ship;
   * MIDDLE: the middle level of the rocket;
   * HIGH: the highest level of the rocket;
   * OVER: dropped into the cargo ship from above
   */
  public enum goalHeight{
    LOW, MIDDLE, HIGH, OVER
  }

  /**
   * different types of goals on the field.
   * CARGO_CARGO: put cargo in the cargo ship;
   * ROCKET_CARGO: put cargo in the rocket;
   * CARGO_HATCH: put a hatch on the cargo ship;
   * ROCKET_HATCH: put a hatch on the rocket;
   * RETRIEVE_HATCH: pick up a hatch from the loading station;
   * RETRIEVE_CARGO: pick up cargo from an unspecified location
   */
  public enum goalType{
    CARGO_CARGO, CARGO_HATCH, ROCKET_CARGO, ROCKET_HATCH, RETRIEVE_HATCH, RETRIEVE_CARGO
  }

  public goalHeight gHeight;
  public goalType gType;
  public heldPiece piece;
  public AutoCommandGroup bigCommandGroup;

  /**
   * generates the command groups based on the inputted goal height/type
   * @param gHeight
   *    the height of the goal the robot should aim for (LOW, MIDDLE, HIGH, OVER)
   * @param gType
   *    the type of goal
   */

  public AutoMotion (goalHeight gHeight, goalType gType){
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
      this.bigCommandGroup = new AutoCommandGroup(genPlaceCommands());
    }else{
      this.bigCommandGroup = new AutoCommandGroup(genGrabCommands());
    }
  }
  /**
   * Generates commands to pick up a piece based on the parameters of the current AutoMotion
   * @return
   *  an ArrayList of commands
   */
  private ArrayList<Command> genGrabCommands(){
    ArrayList<Command> toReturn = new ArrayList<Command>();
    // TODO actually generate commands
    return toReturn;
  }
  
  /**
   * Generates commands to place piece based on the parameters of the current AutoMotion
   * @return
   *  an ArrayList of commands
   */
  private ArrayList<Command> genPlaceCommands(){
    ArrayList<Command> toReturn = new ArrayList<Command>();
    if (gHeight == goalHeight.LOW){
      // Could also align with line
      // TODO find out the actual units for the speed
      toReturn.add(new SetElevatorHeight(14, false));
      toReturn.add(new FollowVisionTarget(1, 20));
    }else{
      //TODO Align with line (IR sensor?)
    }

    // toReturn.add(new PrepareIntake(getElevatorPreset()));

    switch (piece){
      case HATCH:
        // toReturn.add(new PlaceHatch());
      case CARGO:
        if(gHeight == goalHeight.OVER){
          // toReturn.add(new DropCargo(true));
        }else{
          // toReturn.add(new DropCargo(false));
        }
      default:
        break;
    }

    return toReturn;

  }

  /**
   * selects the correct ElevatorPreset from the Elevator subsystems enum based on 
   * the goalHeight, the goalType, and the heldPiece
   */
  private ElevatorPresets getElevatorPreset(){
    switch (this.gType){
      case CARGO_CARGO:
        if (this.gHeight == goalHeight.LOW){
          return ElevatorPresets.CARGO_SHIP_HATCH;
        }else{
          return ElevatorPresets.CARGO_SHIP_WALL;
        }
      case CARGO_HATCH:
        return ElevatorPresets.CARGO_SHIP_HATCH;
      case ROCKET_CARGO:
        if (this.gHeight == goalHeight.LOW){
          return ElevatorPresets.LOW_ROCKET_PORT;
        }else if (gHeight == goalHeight.MIDDLE){
          return ElevatorPresets.MIDDLE_ROCKET_PORT;
        }else{
          return ElevatorPresets.HIGH_ROCKET_PORT;
        }
      case ROCKET_HATCH:
        if (this.gHeight == goalHeight.LOW){
          return ElevatorPresets.LOW_ROCKET_HATCH;
        }else if (this.gHeight == goalHeight.MIDDLE){
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
   * 
   * @return
   *  the goalHeight of the AutoMotion
   */
  public goalHeight getGoalHeight(){
    return this.gHeight;
  }

  /**
   * identification function
   * @return
   *  the goalType of the AutoMotion
   */
  public goalType getGoalType(){
    return this.gType;
  }

  /**
   * identification function
   * @return
   *  the heldPiece of the AutoMotion
   */
  public heldPiece getheldPiece(){
    return this.piece;
  }
  
}
