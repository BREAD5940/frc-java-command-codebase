package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.actions.FollowVisionTarget;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;
import frc.robot.subsystems.Elevator.ElevatorPresets;

import java.util.ArrayList;

/**
 * Creates a command group for a specific automatic motion.
 * Input a type of goal and a height then start the mBigCommandGroup externally
 * 
 * @author Jocelyn McHugo
 */
public class AutoMotion {

  public enum mHeldPiece{
    HATCH, CARGO, NONE
  }
  
  /**
   * different heights of goals.
   * LOW: the lowest level of the rocket and through the hatch of the cargo ship;
   * MIDDLE: the middle level of the rocket;
   * HIGH: the highest level of the rocket;
   * OVER: dropped into the cargo ship from above
   */
  public enum mGoalHeight{
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
  public enum mGoalType{
    CARGO_CARGO, CARGO_HATCH, ROCKET_CARGO, ROCKET_HATCH, RETRIEVE_HATCH, RETRIEVE_CARGO
  }

  public mGoalHeight gHeight;
  public mGoalType gType;
  public mHeldPiece piece;
  public AutoCommandGroup mBigCommandGroup;

  /**
   * generates the command groups based on the inputted goal height/type
   * @param gHeight
   *    the height of the goal the robot should aim for (LOW, MIDDLE, HIGH, OVER)
   * @param gType
   *    the type of goal
   */

  public AutoMotion (mGoalHeight gHeight, mGoalType gType){
    this.gHeight = gHeight;
    this.gType = gType;
    if (gType == mGoalType.CARGO_CARGO || gType == mGoalType.ROCKET_CARGO){
      this.piece = mHeldPiece.CARGO;
    }else if (gType == mGoalType.CARGO_HATCH || gType == mGoalType.ROCKET_HATCH){
      this.piece = mHeldPiece.HATCH;
    }else{
      this.piece=mHeldPiece.NONE;
    }
    if (piece!=mHeldPiece.NONE){
      this.mBigCommandGroup = new AutoCommandGroup(genPlaceCommands());
    }else{
      this.mBigCommandGroup = new AutoCommandGroup(genGrabCommands());
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
    if (gHeight == mGoalHeight.LOW){
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
        if(gHeight == mGoalHeight.OVER){
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
   * the mGoalHeight, the mGoalType, and the mHeldPiece
   */
  private ElevatorPresets getElevatorPreset(){
    switch (this.gType){
      case CARGO_CARGO:
        if (this.gHeight == mGoalHeight.LOW){
          return ElevatorPresets.CARGO_SHIP_HATCH;
        }else{
          return ElevatorPresets.CARGO_SHIP_WALL;
        }
      case CARGO_HATCH:
        return ElevatorPresets.CARGO_SHIP_HATCH;
      case ROCKET_CARGO:
        if (this.gHeight == mGoalHeight.LOW){
          return ElevatorPresets.LOW_ROCKET_PORT;
        }else if (gHeight == mGoalHeight.MIDDLE){
          return ElevatorPresets.MIDDLE_ROCKET_PORT;
        }else{
          return ElevatorPresets.HIGH_ROCKET_PORT;
        }
      case ROCKET_HATCH:
        if (this.gHeight == mGoalHeight.LOW){
          return ElevatorPresets.LOW_ROCKET_HATCH;
        }else if (this.gHeight == mGoalHeight.MIDDLE){
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
   *  the mGoalHeight of the AutoMotion
   */
  public mGoalHeight getGoalHeight(){
    return this.gHeight;
  }

  /**
   * identification function
   * @return
   *  the mGoalType of the AutoMotion
   */
  public mGoalType getmGoalType(){
    return this.gType;
  }

  /**
   * identification function
   * @return
   *  the mHeldPiece of the AutoMotion
   */
  public mHeldPiece getmHeldPiece(){
    return this.piece;
  }
  
}
