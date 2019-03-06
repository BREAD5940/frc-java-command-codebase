package frc.robot.commands.auto;

import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.lib.AutoCommand;
import frc.robot.lib.statemachines.AutoMotionStateMachine;
import frc.robot.planners.SuperstructureMotion;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

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

    //TODO the bit that sets this

    return new SuperStructureState(eState, aState);
  }
}