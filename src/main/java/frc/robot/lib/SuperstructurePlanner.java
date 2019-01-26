package frc.robot.lib;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.commands.subsystems.superstructure.elevator.SetElevatorHeight;
import frc.robot.commands.subsystems.superstructure.wrist.SetWrist;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.superstructure.Wrist;
import frc.robot.subsystems.superstructure.Wrist.WristPos;

/**
 * Plans the best motion of the superstructure based on the inputted current
 * SuperstructureState and a goal SuperstructureState. General idea is to get
 * from point A to point B without breaking anything. TODO find the actual
 * values of the angles/heights. this will probably have to wait until the robot
 * is done
 * 
 * @author Jocelyn McHugo
 */
public class SuperstructurePlanner{

  public SuperstructurePlanner(){}

  //TODO add values for certain elevator positions (ex. the wrist can be <0 if the elevator is >10)

  //TODO get actual irl angles TODO make the names less horrible
  final double minUnCrashHeight=5; //min elevator height + how much intake is below the bottom of the elevator

  final double crossbarHeight = 20;

  boolean intakeCrashable = false; //intake capable of hitting the ground
  boolean intakeAtRisk = false; //intake at risk of hitting the crossbar
  int errorCount; //number of errors in motion
  int corrCount; //number of corrected items in motion

  /**
   * Creates a command group of superstructure motions that will prevent any damage to the intake/elevator
   * @param goalStateIn
   *    the desired SuperstructureState
   * @param currentState
   *    the current SuperstructureState
   * @return
   *    the ideal command group to get from the currentState to the goalState
   */
  public CommandGroup plan(SuperstructureState goalStateIn, SuperstructureState currentState){
    CommandGroup toReturn = new CommandGroup();
    SuperstructureState goalState = new SuperstructureState(goalStateIn);

    if(goalState==currentState){
      System.out.println("MOTION UNNECESSARY -- Goal and current states are same. Exiting planner.");
      return toReturn;
    }

    if(goalState.getHeldPiece()!=currentState.getHeldPiece()){
      System.out.println("MOTION IMPOSSIBLE -- Superstructure motion cannot change heldPiece. Resolving error.");
      errorCount++;
      corrCount++;
      goalState.setHeldPiece(currentState.getHeldPiece());
    }

    if(!goalState.getStanAngle()||!currentState.getStanAngle()){
      System.out.println("MOTION UNSAFE -- Wrist position is wildcard. Setting to default position for movement.");
      errorCount++;
      if(currentState.getHeldPiece()==mHeldPiece.HATCH){
        //TODO change this so it only happens if the intake will ACTUALLY pass through the elevator
        System.out.println("MOTION UNSAFE -- Cannot move wrist to wildcard position while holding hatch. Aborting wrist movement.");
        errorCount++;
        corrCount++;
        goalState.setWristAngle(WristPos.CARGO);
      }else{
        toReturn.addSequential(new SetWrist(WristPos.CARGO));
        intakeAtRisk=false;
        intakeCrashable=false;
      }
    }else{
      // Checks if the intake will ever be inside the elevator
      if((currentState.getWristAngle()==WristPos.HATCH) || (goalState.getWristAngle()==WristPos.HATCH)){
            intakeAtRisk=true;
      }

      //checks if the intake will tilt/is tilted below the bottom of the elevator
      if((goalState.getWristAngle()==WristPos.DOWN) ||(currentState.getWristAngle()==WristPos.DOWN)){
        intakeCrashable=true;
      }
    }

    //checks if the elevator will move past the crossbar
    if(intakeAtRisk&&(goalState.getElevatorHeight()>=crossbarHeight&&currentState.getElevatorHeight()<=crossbarHeight)
        || (goalState.getElevatorHeight()<=crossbarHeight&&currentState.getElevatorHeight()>=crossbarHeight)){
      if(currentState.getHeldPiece()==mHeldPiece.HATCH){
        System.out.println("MOTION UNSAFE -- Intake will hit crossbar and cannot be moved. Moving to max possible height.");
        errorCount++;
        corrCount++;
        goalState.setElevatorHeight(crossbarHeight-1.5); //crossbarHeight - how much intake there is inside the elevator TODO confirm
      }else{
        System.out.println("MOTION UNSAFE -- Intake will hit crossbar. Setting to default intake position for movement.");
        errorCount++;
        toReturn.addSequential(new SetWrist(WristPos.CARGO)); //Keeps intake outside the elevator so it doesn't hit the crossbar
      }
    }else{
      intakeAtRisk=false;
    }
    
    //checks if the elevator will move low enough to crash the intake
    if (goalState.getElevatorHeight()<=minUnCrashHeight&&intakeCrashable){
      System.out.println("MOTION UNSAFE -- Intake will hit ground. Setting to default intake position.");
      errorCount++;
      corrCount++;
      goalState.setWristAngle(Wrist.WristPos.CARGO);
    }else{
      intakeCrashable=false;
    }

    //move to corrected state
    toReturn.addSequential(new SetElevatorHeight(goalState.getElevatorHeight()));
    currentState.setElevatorHeight(goalState.getElevatorHeight());
    if(goalState.getStanAngle()){
      toReturn.addSequential(new SetWrist(goalState.getWristAngle()));
      currentState.setWristAngle(goalState.getWristAngle());
    }else{
      toReturn.addSequential(new SetWrist(goalState.getRawWristAngle()));
      currentState.setWristAngle(goalState.getRawWristAngle());
    }

    // current and goal should now be equal
    if(currentState==goalState){ //TODO check if this throws errors with preset vs. raw values
      System.out.println("MOTION COMPLETED -- "+Integer.valueOf(errorCount)+" error(s) and "
        +Integer.valueOf(corrCount)+" final correction(s)\n");
      return toReturn;
    }else{
      System.out.println("MOTION FAILED -- Final states not equal.");
      return null;
    }
  }
}