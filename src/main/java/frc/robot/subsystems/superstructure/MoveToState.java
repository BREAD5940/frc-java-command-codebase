package frc.robot.subsystems.superstructure;

import frc.robot.Robot;
import frc.robot.states.SuperstructureState;

public class MoveToState {

  protected MoveToState(){}

  public void move(SuperstructureState state){
    Robot.superstructure.wrist.setAngle(state.getAngle());
    Robot.superstructure.elevator.setHeight(state.getElevatorHeight());
  }
}