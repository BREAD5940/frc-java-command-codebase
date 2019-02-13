package frc.robot.states;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Rotation2d;

import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.lib.Loggable;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

public class SuperStructureState implements Loggable {
	public IntakeAngle jointAngles;
	public ElevatorState elevator;
	private HeldPiece piece = HeldPiece.NONE;

	public SuperStructureState() {
		this(new ElevatorState(), new RotatingArmState(), new RotatingArmState(), HeldPiece.NONE);
	}

	public SuperStructureState(SuperStructureState other) {
		this(other.elevator, other.jointAngles);
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
		this(elevatorState, intakeState, HeldPiece.NONE);
	}

	public SuperStructureState(ElevatorState elevatorState, IntakeAngle angles___, HeldPiece piece__) {
		jointAngles = angles___;
		elevator = elevatorState;
		piece = piece__;
	}

	public static SuperStructureState fromOther(SuperStructureState other) {
		return new SuperStructureState(other.elevator, other.jointAngles, other.piece);
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
		return new SuperStructureState(elevator.add(delta), jointAngles);
	}

	public void setAngle(IntakeAngle angle_) {
		this.jointAngles = angle_;
	}

	public IntakeAngle getAngle() {
		return jointAngles;
	}

	public boolean isEqualTo(SuperStructureState other) {
		System.out.printf("Joint angles: %b   Elevator: %b   Piece: %b\n",jointAngles.isEqualTo(other.jointAngles),
				elevator.isEqualTo(other.elevator),piece.toString().equals(other.piece.toString()));
		return (this.jointAngles.isEqualTo(other.jointAngles)
				&& this.elevator.isEqualTo(other.elevator)
				&& this.piece.toString() == other.piece.toString());
	}

	@Override
	public String toString(){
		return "HeldPiece: "+this.piece.toString()+", Elbow Angle: "+this.jointAngles.getElbow().toString()+
						", Wrist Angle: "+this.jointAngles.getWrist().toString()+", Elevator Height: "+this.elevator.getHeight().getFeet();
	}

	@Override
	public String getCSVHeader() {
		return "Elevator,Elbow,Wrist,Piece";
	}

	@Override
	public String toCSV() {
		return elevator.getHeight().getInch() + "," + jointAngles.getElbow().angle.getDegree() + ","
				+ jointAngles.getWrist().angle.getDegree() + "," + piece.name();
	}

}
