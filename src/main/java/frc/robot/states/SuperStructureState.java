package frc.robot.states;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Rotation2d;

import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.lib.obj.SuperStructureJoint;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

public class SuperStructureState {
  public IntakeAngle jointAngles;
  public ElevatorState elevator;
  private HeldPiece piece = HeldPiece.NONE; // FIXME because a lowercase first letter breaks syntax highlighting

  public SuperStructureState() {
    this(new ElevatorState(), new RotatingArmState(), new RotatingArmState(), HeldPiece.NONE);
  }

  public SuperStructureState(ElevatorState elevatorState, RotatingArmState elbowState, RotatingArmState wristState, HeldPiece heldPiece) {
    jointAngles = new IntakeAngle(elbowState, wristState);
    elevator = elevatorState;
    piece = heldPiece;
  }

  // public SuperStructureState(ElevatorState elevatorState, IntakeAngle intakeState)

  public SuperStructureState(ElevatorState elevatorState, RotatingArmState elbowState, RotatingArmState wristState) {
    jointAngles = new IntakeAngle(elbowState, wristState);
    elevator = elevatorState;
    piece = HeldPiece.NONE;
  }

  public SuperStructureState(ElevatorState elevatorState, IntakeAngle intakeState) {
    this(intakeState, elevatorState, HeldPiece.NONE);
  }

  public SuperStructureState(IntakeAngle angles___, ElevatorState elevatorState, HeldPiece piece__) {
    jointAngles = angles___;
    elevator = elevatorState;
    piece = piece__;
  }

  public static SuperStructureState fromOther(SuperStructureState other) {
    return new SuperStructureState( other.jointAngles, other.elevator, other.piece );
  }

  public HeldPiece getHeldPiece() {
    return piece;
  }

  public void setHeldPiece(HeldPiece new_) {
    piece = new_;
  }

  public Rotation2d getElbowAngle() {
    return jointAngles.getElbow().angle;
  }

  public void setElbowAngle(Rotation2d newAngle) {
    jointAngles.getElbow().setAngle(newAngle);
  }

  public Length getElevatorHeight() {
    return elevator.height;
  }

  public ElevatorState getElevator() {
    return elevator;
  }

  public RotatingArmState getWrist() {
    return jointAngles.getWrist();
  }

  public RotatingArmState getElbow() {
    return jointAngles.getElbow();
  }

  public SuperStructureState addElevator(Length delta) {
    return new SuperStructureState( elevator.add(delta), jointAngles );
  }

  public void setAngle(IntakeAngle angle_) {
    this.jointAngles = angle_;
  }

  public IntakeAngle getAngle() {
    return jointAngles;
  }
}
 