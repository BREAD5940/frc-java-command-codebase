package frc.robot.states;

import org.ghrobotics.lib.mathematics.units.Length;

import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.lib.Loggable;
import frc.robot.lib.obj.RoundRotation2d;
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

	public HeldPiece getHeldPiece() {
		return piece;
	}

	public void setHeldPiece(HeldPiece new_) {
		piece = new_;
	}

	public RoundRotation2d getElbowAngle() {
		return jointAngles.getElbow().angle;
	}

	public RoundRotation2d getWristAngle() {
		return jointAngles.getWrist().angle;
	}

	public void setElbowAngle(RoundRotation2d newAngle) {
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
		return (this.jointAngles.isEqualTo(other.jointAngles)
				&& this.elevator.isEqualTo(other.elevator)
				&& this.piece.toString() == other.piece.toString());
	}

	@Override
	public String toString() {
		return "HeldPiece: " + this.piece.toString() + ", Elbow Angle: " + this.jointAngles.getElbow().toString() +
				", Wrist Angle: " + this.jointAngles.getWrist().toString() + ", Elevator Height: " + this.elevator.getHeight().getFeet();
	}

	@Override
	public String getCSVHeader() {
		return "Elevator,Elbow,Wrist,Piece";
	}

	@Override
	public String toCSV() {
		return Math.round(elevator.getHeight().getInch() * 100) / 100 + "," + Math.round(jointAngles.getElbow().angle.getDegree() * 100) / 100 + ","
				+ Math.round(jointAngles.getWrist().angle.getDegree() * 100) / 100 + "," + piece.name();
	}

}
