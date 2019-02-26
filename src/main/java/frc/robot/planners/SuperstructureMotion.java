package frc.robot.planners;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Optional;

import javax.annotation.processing.RoundEnvironment;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.SuperStructureConstants;
import frc.robot.commands.subsystems.superstructure.ArmMove;
import frc.robot.commands.subsystems.superstructure.ElevatorMove;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;


/**
 * Instructions for using this mutant command-thing:
 *  - Do NOT call the constructor
 *  - Call the planner
 *  - Call SuperstructureMotion.getInstance().start();
 *    - This will iterate through the planned commandQueue
 *    - It'll end when it's done
 */
public class SuperstructureMotion extends Command {
  /* RELEVANT COMMANDS:
      - ElevatorMove
      - ArmMove
      - ArmWaitForElevator
   */


  private SuperstructureMotion() {
    requires(SuperStructure.getInstance());
  }

  
  private SuperstructureMotion instance_;
  protected LinkedList<Command> queue = new LinkedList<Command>();
  protected Optional<Command> current;



  public SuperstructureMotion getInstance(){
    if (instance_ == null) {
      instance_ = new SuperstructureMotion();
    }
    return instance_;
  }


  public boolean plan(SuperStructureState gsIn, SuperStructureState currentState){
    SuperStructureState goalState = new SuperStructureState(gsIn);

    //CHECK if the current and goal match
    if(goalState.isEqualTo(currentState)){
      queue.add(new ArmMove());
      queue.add(new ElevatorMove());
      return true;
    }
    //SAFE illegal inputs
    if(goalState.getElevatorHeight().getInch() > SuperStructureConstants.Elevator.top.getInch()){
      goalState.getElevator().setHeight(SuperStructureConstants.Elevator.top);
    } else if (goalState.getElevatorHeight().getInch() < SuperStructureConstants.Elevator.bottom.getInch()){
      goalState.getElevator().setHeight(SuperStructureConstants.Elevator.bottom);
    }

    if(goalState.getElbowAngle().getDegree() > SuperStructureConstants.Elbow.kElbowMax.getDegree()){
      goalState.getElbow().setAngle(SuperStructureConstants.Elbow.kElbowMax);
    } else if (goalState.getElbowAngle().getDegree() < SuperStructureConstants.Elbow.kElbowMin.getDegree()){
      goalState.getElbow().setAngle(SuperStructureConstants.Elbow.kElbowMin);
    }

    if(goalState.getWristAngle().getDegree() > SuperStructureConstants.Wrist.kWristMax.getDegree()){
      goalState.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMax);
    }else if (goalState.getWristAngle().getDegree() < SuperStructureConstants.Wrist.kWristMin.getDegree()){
      goalState.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMin);
    }

    //DEFINE the three goal points -- elevator, wrist, and end of intake
    Translation2d GPelevator  = new Translation2d(LengthKt.getInch(0), goalState.getElevatorHeight());
    Translation2d GPwrist = new Translation2d(LengthKt.getInch(goalState.getElbowAngle().getCos()*SuperStructureConstants.Elbow.carriageToIntake.getInch()),
            LengthKt.getInch(goalState.getElbowAngle().getSin()*SuperStructureConstants.Elbow.carriageToIntake.getInch()).plus(GPelevator.getY()));
    Translation2d GPeoi = new Translation2d(LengthKt.getInch(goalState.getWristAngle().getCos()*SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getX()),
            LengthKt.getInch(goalState.getWristAngle().getSin()*SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getY()));
    //DEFINE the three start points -- elevator, wrist, and end of intake
    Translation2d SPelevator = new Translation2d(LengthKt.getInch(0), currentState.getElevatorHeight());
    Translation2d SPwrist = new Translation2d(LengthKt.getInch(currentState.getElbowAngle().getCos()*SuperStructureConstants.Elbow.carriageToIntake.getInch()),
            LengthKt.getInch(currentState.getElbowAngle().getSin()*SuperStructureConstants.Elbow.carriageToIntake.getInch()).plus(SPelevator.getY()));
    Translation2d SPeoi = new Translation2d(LengthKt.getInch(currentState.getWristAngle().getCos()*SuperStructureConstants.Wrist.intakeOut.getInch()).plus(SPwrist.getX()),
    LengthKt.getInch(currentState.getWristAngle().getSin()*SuperStructureConstants.Wrist.intakeOut.getInch()).plus(SPwrist.getY()));

    //SAFE potential crashes on the end state
    
    if(GPeoi.getY().getInch() < SuperStructureConstants.electronicsHeight.getInch()){
      RoundRotation2d tempTheta = goalState.getWristAngle();
      tempTheta = RoundRotation2d.getRadian(Math.asin((GPeoi.getY().getInch()-GPwrist.getY().getInch())/SuperStructureConstants.Wrist.intakeOut.getInch()));
      goalState.getWrist().setAngle(tempTheta);
      GPeoi = new Translation2d(GPeoi.getX(),  LengthKt.getInch(Math.sin(tempTheta.getRadian())*SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getY()));
    }

    if(GPwrist.getY().getInch() < SuperStructureConstants.electronicsHeight.getInch()){
      RoundRotation2d tempTheta = goalState.getElbowAngle();
      tempTheta = RoundRotation2d.getRadian(Math.asin((GPwrist.getY().getInch()-GPelevator.getY().getInch())/SuperStructureConstants.Elbow.carriageToIntake.getInch()));
      goalState.getElbow().setAngle(tempTheta);
      GPwrist = new Translation2d(GPwrist.getX(),  LengthKt.getInch(Math.sin(tempTheta.getRadian())*SuperStructureConstants.Elbow.carriageToIntake.getInch()).plus(GPelevator.getY()));

    }

    
    

    //CLEAR the queue

    
    //CHECK the position of the intake -- hatch or cargo
    // IF it's a long climb
      //ADD intake stowage to the queue
      //ADD a commandoncondition to the queue to reset the intake when the elevator is within tolerance
    //CHECK if the elevator point is in proximity to the crossbar
      //STOW the intake if it's in danger
    //


    return true;
  }



  public LinkedList<Command> getQueue(){
    return this.queue;
  }


  // public Command getCurrent(){
  //   return this.current;
  // }

  public void setCurrent(int queuePos){
    // this.current = this.queue.get(queuePos);
  }

  @Override
  protected boolean isFinished() {
    boolean itBeDone = true;
    for(int i=0; i<this.queue.size(); i++){
      itBeDone = this.queue.get(i).isCompleted() && itBeDone;
    }

    return itBeDone;
  }

  

}