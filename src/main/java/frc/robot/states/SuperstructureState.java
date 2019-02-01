package frc.robot.states;

import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

public class SuperStructureState {
  public RotatingArmState elbow; // TODO add elevator accleration
  public RotatingArmState wrist;
  public ElevatorState elevator;
  public mHeldPiece piece = mHeldPiece.NONE; // FIXME because a lowercase first letter breaks syntax highlighting

  public SuperStructureState() {
    this(new RotatingArmState(), new RotatingArmState(), new ElevatorState(), mHeldPiece.NONE);
  }

  public SuperStructureState(RotatingArmState elbowState, RotatingArmState wristState, ElevatorState elevatorState, mHeldPiece heldPiece) {
    elbow = elbowState;
    wrist = wristState;
    elevator = elevatorState;
    piece = heldPiece;
  }

  public SuperStructureState(RotatingArmState elbowState, RotatingArmState wristState, ElevatorState elevatorState) {
    elbow = elbowState;
    wrist = wristState;
    elevator = elevatorState;
    piece = mHeldPiece.NONE;
  }

  public static SuperStructureState fromOther(SuperStructureState other) {
    return new SuperStructureState( other.elbow, other.wrist, other.elevator, other.piece )
  }
}