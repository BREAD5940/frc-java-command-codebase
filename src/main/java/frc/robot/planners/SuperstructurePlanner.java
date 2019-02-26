package frc.robot.planners;

import java.util.ArrayList;
import java.util.LinkedList;

import frc.robot.states.SuperStructureState;

public class SuperstructurePlanner{
  private SuperstructurePlanner(){}
  private SuperstructurePlanner instance_;
  protected LinkedList<SuperStructureState> queue = new LinkedList<>();
  protected SuperStructureState current;



  public SuperstructurePlanner getInstance(){
    if (instance_ == null){
      instance_ = new SuperstructurePlanner();
    }
    return instance_;
  }


  public boolean plan(SuperStructureState gsIn, SuperStructureState currentState){
    SuperStructureState goalState = new SuperStructureState(gsIn);

    //CHECK if the current and goal match
    //SAFE illegal inputs

    //CLEAR the queue

    //DEFINE the three goal points -- elevator, wrist, and end of intake
    //DEFINE the three start points -- elevator, wrist, and end of intake
    //CHECK the position of the intake -- hatch or cargo
    // IF it's a long climb
      //ADD intake stowage to the queue
      //ADD a commandoncondition to the queue to reset the intake when the elevator is within tolerance
    //CHECK if the elevator point is in proximity to the crossbar
      //STOW the intake if it's in danger
    //


    return true;
  }



  public LinkedList<SuperStructureState> getQueue(){
    return this.queue;
  }


  public SuperStructureState getCurrent(){
    return this.current;
  }

  public void setCurrent(int queuePos){
    this.current = this.queue.get(queuePos);
  }

  

}