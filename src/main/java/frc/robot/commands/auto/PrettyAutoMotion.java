package frc.robot.commands.auto;

import frc.robot.RobotConfig;
import frc.robot.RobotConfig.auto;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.lib.AutoCommand;
import frc.robot.lib.statemachines.AutoMotionStateMachine;
import frc.robot.planners.SuperstructureMotion;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PrettyAutoMotion{

  private AutoCommandGroup motionCommands;
  private AutoCommandGroup presetCommands;
  private SuperStructureState ssState;

  public PrettyAutoMotion(AutoMotionStateMachine machine){
    this.ssState = findSuperStructureState(machine);
    this.presetCommands = genPresetMotion(this.ssState);
    this.motionCommands = genMainMotion(machine);
  }

  private AutoCommandGroup genMainMotion(AutoMotionStateMachine machine){
    AutoCommandGroup createdGroup = new AutoCommandGroup();

    //TODO prepare for the never-ending nest of switches

    return createdGroup;
  }

  private AutoCommandGroup genPresetMotion(SuperStructureState state){
    SuperstructureMotion.getInstance().plan(state, SuperStructure.getInstance().lastState); //FIXME lastState is what we want, right?
    return SuperstructureMotion.getInstance().getQueue();
  }

  private SuperStructureState findSuperStructureState(AutoMotionStateMachine machine){
    ElevatorState eState = new ElevatorState();
    IntakeAngle aState = new IntakeAngle(new RotatingArmState(), new RotatingArmState());

    //FIXME all the fieldPositions need tuning
    switch (machine.getGoalLocation()){
      case CARGO_SHIP:
        switch (machine.getHeldPiece()){
          case CARGO:
            eState.setHeight(RobotConfig.auto.fieldPositions.shipWall);
            aState = iPosition.CARGO_DOWN; //FIXME change this angle
            break;
          case HATCH:
            eState.setHeight(RobotConfig.auto.fieldPositions.hatchLowGoal);
            aState = iPosition.HATCH; //FIXME confirm
            break;
          case NONE:
            //no
            break;
        }
      case ROCKET:
        switch (machine.getHeldPiece()){
          case CARGO:
            //TODO add a way to pick the specific cargo position to the sm
            aState = iPosition.CARGO_PLACE;
            switch (machine.getGoalHeight()){
              case LOW:
                eState.setHeight(auto.fieldPositions.cargoLowGoal);
                break;
              case MIDDLE:
                eState.setHeight(fieldPositions.cargoMiddleGoal);
                break;
              case HIGH:
                eState.setHeight(fieldPositions.cargoHighGoal);
                break;
            }
          case HATCH:
            //TODO see above
            aState = iPosition.HATCH;
            switch(machine.getGoalHeight()){
              case LOW:
                eState.setHeight(fieldPositions.hatchLowGoal);
                break;
              case MIDDLE:
                eState.setHeight(fieldPositions.hatchMiddleGoal);
                break;
              case HIGH:
                eState.setHeight(fieldPositions.hatchHighGoal);
                break;
            }
          case NONE:
            //no
            break;
        }
      case LOADING:
        switch (machine.getGoalPiece()){
          case HATCH:
            eState.setHeight(fieldPositions.hatchLowGoal);
            //FIXME see above
            aState = iPosition.HATCH_PITCHED_UP;
            break;
          case CARGO:
            //TODO this will do a thing eventually
            break;
          case NONE:
            //no
            break;
        }
      case DEPOT:
        //FIXME honestly I don't think this is actually gonna be a thing
        break;
    }

    return new SuperStructureState(eState, aState);
  }
}