package frc.robot.commands.auto;

import frc.robot.commands.auto.actions.AutoIntake;
import frc.robot.commands.auto.actions.DriveDistance;
import frc.robot.commands.auto.actions.SetIntakeMode;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.GrabCargo;
import frc.robot.commands.auto.groups.GrabHatch;
import frc.robot.commands.auto.groups.PlaceHatch;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTarget;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;
import frc.robot.subsystems.Elevator.ElevatorPresets;

/**
 * Creates a command group for a specific automatic motion. Input a type of goal
 * and a height then start the mBigCommandGroup externally In the future, this
 * could change to more inputs depending on the button setup
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

  private mGoalHeight gHeight;
  private mGoalType gType;
  private mHeldPiece piece;
  private AutoCommandGroup mBigCommandGroup;

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
    //select heldPiece
    if (this.gType == mGoalType.CARGO_CARGO || this.gType == mGoalType.ROCKET_CARGO){
      this.piece = mHeldPiece.CARGO;
    }else if (this.gType == mGoalType.CARGO_HATCH || this.gType == mGoalType.ROCKET_HATCH){
      this.piece = mHeldPiece.HATCH;
    }else{
      this.piece=mHeldPiece.NONE;
    }

    if (this.piece!=mHeldPiece.NONE){
      this.mBigCommandGroup = genPlaceCommands();
    }else{
      this.mBigCommandGroup = genGrabCommands();
    }
  }
  /**
   * Generates commands to pick up a piece based on the parameters of the current AutoMotion
   * @return
   *  an ArrayList of commands
   */
  private AutoCommandGroup genGrabCommands(){
    AutoCommandGroup toReturn = new AutoCommandGroup();
    if (this.gType==mGoalType.RETRIEVE_CARGO){
      // Set the intake to cargo mode
      toReturn.addSequential(new SetIntakeMode(mHeldPiece.CARGO));
      // Predefined grab command
      toReturn.addSequential(new GrabCargo());
    }else if (this.gType==mGoalType.RETRIEVE_HATCH){
      // Set the intake to hatch mode
      toReturn.addSequential(new SetIntakeMode(mHeldPiece.HATCH));
      // Predefined grab command
      toReturn.addSequential(new GrabHatch());
    }
    return toReturn;
  }

  /**
   * Generates commands to place a piece based on the parameters of the current AutoMotion
   * @return
   *  an ArrayList of commands
   */
  private AutoCommandGroup genPlaceCommands(){
    AutoCommandGroup toReturn = new AutoCommandGroup();

    // Set intake mode
    if (this.gType==mGoalType.CARGO_CARGO){
      toReturn.addSequential(new SetIntakeMode(this.piece, true));
    }else{
      toReturn.addSequential(new SetIntakeMode(this.piece));
    }

    // Align with the vision targets, slightly back from the goal
    toReturn.addSequential(new FollowVisionTarget(0.7, 70, 20)); // FIXME check % value TODO this assumes a perfect FollowVisionTarget command

    // Set the elevator to the correct height
    toReturn.addSequential(new SetElevatorHeight(getElevatorPreset(),false));

    if(this.gType==mGoalType.CARGO_CARGO){
      // Drive forward so the intake is over the bay and the bumpers are in the indent thingy
      toReturn.addSequential(new DriveDistance(2,20)); // FIXME check distances
    }else{
      // Drive forward so the intake is flush with the port/hatch
      toReturn.addSequential(new DriveDistance(1,20)); // FIXME check distances
    }

    if(this.piece==mHeldPiece.CARGO){
      toReturn.addSequential(new AutoIntake(-1, 5));
    }else if (this.piece==mHeldPiece.HATCH){
      toReturn.addSequential(new PlaceHatch());
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

  /**
   * identification function
   * @return
   *  the mBigCommandGroup of the function
   */
  public AutoCommandGroup getBigCommandGroup(){
    return this.mBigCommandGroup;
  }
  
}
