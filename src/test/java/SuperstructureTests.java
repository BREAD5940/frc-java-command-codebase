import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.Arrays;

import org.junit.jupiter.api.Test;

import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.superstructure.Wrist.WristPos;

public class SuperstructureTests {
  

  @Test
  public void testWrists(){
    SuperstructurePlanner planner = new SuperstructurePlanner();
    SuperstructureState currentState = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE);
    ArrayList<SuperstructureState> goalStates = new ArrayList<SuperstructureState>(Arrays.asList(
              new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE),
              new SuperstructureState(0,WristPos.HATCH,mHeldPiece.NONE),
              new SuperstructureState(0,WristPos.DOWN,mHeldPiece.NONE),
              new SuperstructureState(0,10,mHeldPiece.NONE),
              new SuperstructureState(0,WristPos.CARGO,mHeldPiece.CARGO)));
    ArrayList<SuperstructureState> correctEndStates = new ArrayList<SuperstructureState>(Arrays.asList(
              new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE),
              new SuperstructureState(0,WristPos.HATCH,mHeldPiece.NONE),
              new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE),
              new SuperstructureState(0,10,mHeldPiece.NONE),
              new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE)));
    ArrayList<SuperstructureState> resultingStates=new ArrayList<SuperstructureState>();


    for(int i=0; i<goalStates.size(); i++){
      resultingStates.add(i,planner.getPlannedState(goalStates.get(i),currentState));
      System.out.print(Integer.valueOf(i)+": ");
      if(resultingStates.get(i).getStanAngle()&&correctEndStates.get(i).getStanAngle()){
        System.out.print(correctEndStates.get(i).getWristAngle().angle1);
        System.out.print(", ");
        System.out.println(resultingStates.get(i).getWristAngle().angle1);
        assertEquals(correctEndStates.get(i).getWristAngle().angle1, resultingStates.get(i).getWristAngle().angle1);
      }else{
        System.out.print(correctEndStates.get(i).getRawWristAngle());
        System.out.print(", ");
        System.out.println(resultingStates.get(i).getRawWristAngle());
        assertEquals(correctEndStates.get(i).getRawWristAngle(), resultingStates.get(i).getRawWristAngle()); 
      }
    }
    System.out.println();
  }

  @Test
  public void testElevator(){
    SuperstructurePlanner planner = new SuperstructurePlanner();
    SuperstructureState currentState = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE);
    ArrayList<SuperstructureState> goalStates = new ArrayList<SuperstructureState>(Arrays.asList(
              new SuperstructureState(10,WristPos.CARGO,mHeldPiece.NONE),
              new SuperstructureState(30,WristPos.CARGO,mHeldPiece.NONE),
              new SuperstructureState(80,WristPos.CARGO,mHeldPiece.NONE)));
    ArrayList<SuperstructureState> correctEndStates = new ArrayList<SuperstructureState>(Arrays.asList(
              new SuperstructureState(10,WristPos.CARGO,mHeldPiece.NONE),
              new SuperstructureState(30,WristPos.CARGO,mHeldPiece.NONE),
              new SuperstructureState(70,WristPos.CARGO,mHeldPiece.NONE)));
    ArrayList<SuperstructureState> resultingStates=new ArrayList<SuperstructureState>();


    for(int i=0; i<goalStates.size(); i++){
      resultingStates.add(i,planner.getPlannedState(goalStates.get(i),currentState));
      System.out.print(Integer.valueOf(i)+": ");
      System.out.print(correctEndStates.get(i).getElevatorHeight());
      System.out.print(", "); 
      System.out.println(resultingStates.get(i).getElevatorHeight());
      assertEquals(correctEndStates.get(i).getElevatorHeight(), resultingStates.get(i).getElevatorHeight()); 
    }
    System.out.println();
  }

  @Test
  public void bigScaryComboTests(){
    SuperstructurePlanner planner = new SuperstructurePlanner();
    SuperstructureState currentState = new SuperstructureState(0,WristPos.CARGO,mHeldPiece.NONE);
    ArrayList<SuperstructureState> goalStates = new ArrayList<SuperstructureState>(Arrays.asList(
              new SuperstructureState(10,WristPos.HATCH,mHeldPiece.NONE),
              new SuperstructureState(30,WristPos.HATCH,mHeldPiece.NONE),
              new SuperstructureState(30,10,mHeldPiece.NONE)));
    ArrayList<SuperstructureState> correctEndStates = new ArrayList<SuperstructureState>(Arrays.asList(
              new SuperstructureState(10,WristPos.HATCH,mHeldPiece.NONE),
              new SuperstructureState(30,WristPos.HATCH,mHeldPiece.NONE),
              new SuperstructureState(30,10,mHeldPiece.NONE)));
    ArrayList<SuperstructureState> resultingStates=new ArrayList<SuperstructureState>();


    for(int i=0; i<goalStates.size(); i++){
      resultingStates.add(i,planner.getPlannedState(goalStates.get(i),currentState));

      System.out.print(Integer.valueOf(i)+" heights: ");
      System.out.print(correctEndStates.get(i).getElevatorHeight());
      System.out.print(", "); 
      System.out.println(resultingStates.get(i).getElevatorHeight());
      assertEquals(correctEndStates.get(i).getElevatorHeight(), resultingStates.get(i).getElevatorHeight()); 

      System.out.print(Integer.valueOf(i)+" angles: ");
      if(resultingStates.get(i).getStanAngle()&&correctEndStates.get(i).getStanAngle()){
        System.out.print(correctEndStates.get(i).getWristAngle().angle1);
        System.out.print(", ");
        System.out.println(resultingStates.get(i).getWristAngle().angle1);
        assertEquals(correctEndStates.get(i).getWristAngle().angle1, resultingStates.get(i).getWristAngle().angle1);
      }else{
        System.out.print(correctEndStates.get(i).getRawWristAngle());
        System.out.print(", ");
        System.out.println(resultingStates.get(i).getRawWristAngle());
        assertEquals(correctEndStates.get(i).getRawWristAngle(), resultingStates.get(i).getRawWristAngle()); 
      }
    }
    System.out.println();
  }
}