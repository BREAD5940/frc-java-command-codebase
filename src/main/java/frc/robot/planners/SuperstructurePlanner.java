package frc.robot.planners;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import frc.robot.RobotConfig;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * Plans the best motion of the superstructure based on the inputted current
 * SuperStructureState and a goal SuperStructureState. General idea is to get
 * from point A to point B without breaking anything. TODO find the actual
 * values of the angles/heights. this will probably have to wait until the robot
 * is done
 * 
 * @author Jocelyn McHugo
 */
public class SuperstructurePlanner {

	public SuperstructurePlanner() {}
	//TODO get actual irl angles amd heights

	static final Length bottom = LengthKt.getInch(RobotConfig.elevator.elevator_minimum_height);
	static final Length top = RobotConfig.elevator.elevator_maximum_height;
	static final Length crossbar = LengthKt.getInch(35); //FIXME verify
	static final Length carriageToIntake = LengthKt.getInch(12); //FIXME verify
	static final Length intakeDiameter = LengthKt.getInch(6);

	static final Rotation2d minAboveAngle = Rotation2dKt.getDegree(55); //FIXME verify
	static final Rotation2d maxAboveAngle = Rotation2dKt.getDegree(125);//FIXME verify
	static final Rotation2d minBelowAngle = Rotation2dKt.getDegree(235); //FIXME verify
	static final Rotation2d maxBelowAngle = Rotation2dKt.getDegree(305); //FIXME verify

	Length minSafeHeight = bottom;
	Length maxSafeHeight = top;
	Length disInside = LengthKt.getInch(carriageToIntake.getFeet() + intakeDiameter.getFeet());

	Length gHeight = bottom;

	boolean throughBelow = false;
	boolean throughAbove = false;
	boolean goingToCrash = false;

	int errorCount; //number of errors in motion
	int corrCount; //number of corrected items in motion
	SuperStructureState currentPlannedState;

	public boolean checkValidState(SuperStructureState reqState) { //what is this supposed to do? does it just check if the path is possible w/o correction?
		// TODO is this what we actuall want this to looke like?
		ArrayList<SuperStructureState> kdsjfl = this.plan(reqState, this.currentPlannedState);
		return (kdsjfl.get(kdsjfl.size() - 1).isEqualTo(reqState) && kdsjfl.size() == 1);
		//so now it just checks to see if it can move from the current planned state to the reqstate w/ no correction
	}

	/**
	 * Creates a command group of superstructure motions that will prevent any damage to the intake/elevator
	 * @param goalStateIn
	 *    the desired SuperStructureState
	 * @param currentState
	 *    the current SuperStructureState
	 * @return
	 *    the ideal command group to get from the currentState to the goalState
	 */
	public ArrayList<SuperStructureState> plan(SuperStructureState goalStateIn, SuperStructureState currentState) {
		ArrayList<SuperStructureState> toReturn = new ArrayList<SuperStructureState>();
		SuperStructureState goalState = new SuperStructureState(goalStateIn);
		errorCount = corrCount = 0;

		//heights
		gHeight = LengthKt.getInch((goalState.getElevatorHeight().getInch() + Math.sin(goalState.getAngle().getElbow().angle.getRadian()) / carriageToIntake.getInch())
				+ Math.sin(goalState.getAngle().getWrist().angle.getRadian()) / intakeDiameter.getInch());
		disInside = LengthKt.getInch(Math.abs(gHeight.getInch() - goalState.getElevatorHeight().getInch()));

		//le booleans
		throughBelow = ((currentState.getAngle().getElbow().angle.getRadian() > minBelowAngle.getRadian()
				&& currentState.getAngle().getElbow().angle.getRadian() < maxBelowAngle.getRadian()));
		throughAbove = ((currentState.getAngle().getElbow().angle.getRadian() > minAboveAngle.getRadian()
				&& currentState.getAngle().getElbow().angle.getRadian() < maxAboveAngle.getRadian()));
		goingToCrash = (throughBelow && goalState.getElevator().getHeight().getFeet() < disInside.getFeet())
				|| (throughAbove && goalState.getElevator().getHeight().getFeet() + disInside.getFeet() > top.getFeet());

		if (goalState == currentState) {
			System.out.println("MOTION UNNECESSARY -- Goal and current states are same. Exiting planner.");
			this.currentPlannedState = goalState;
			return toReturn;
		}

		if (goalState.getHeldPiece() != currentState.getHeldPiece()) {
			System.out.println("MOTION IMPOSSIBLE -- Superstructure motion cannot change heldPiece. Resolving error.");
			errorCount++;
			corrCount++;
			goalState.setHeldPiece(currentState.getHeldPiece());
		}

		if (gHeight.getFeet() < bottom.getFeet() + disInside.getFeet()) {
			System.out.println("MOTION IMPOSSIBLE -- Intake will hit literally all of the electronics. Safing elbow angle.");
			errorCount++;
			//finds the necessary angle of the elbow to safe the intake
			double cTheta = Math.asin(carriageToIntake.getInch() * ((bottom.getInch() + disInside.getInch()) - goalState.getElevatorHeight().getInch() -
					Math.sin(goalState.getAngle().getWrist().angle.getRadian())));
			//checks if the adjustment is too big
			if (cTheta < goalState.getAngle().getElbow().angle.getRadian() && cTheta != 0) {
				System.out.println("MOTION UNSAFE -- Angle cannot be safed with only elbow. Safing wrist.");
				//sets the elbow to the max allowed adjustment
				goalState.setElbowAngle(Rotation2dKt.getRadian(0));
				//finds the necessary wrist angle to finish safing the intake
				cTheta = Math.asin(intakeDiameter.getInch() * ((bottom.getInch() + disInside.getInch()) - goalState.getElevatorHeight().getInch() -
						Math.sin(goalState.getAngle().getElbow().angle.getRadian())));
				//sets the wrist to that angle
				goalState.getAngle().getWrist().setAngle(Rotation2dKt.getRadian(cTheta));
				corrCount += 2;
			} else {
				goalState.setElbowAngle(Rotation2dKt.getRadian(cTheta));
				corrCount++;
			}
		}

		if (throughAbove && gHeight.getFeet() > top.getFeet() - disInside.getFeet()) {
			System.out.println("MOTION UNSAFE -- Intake will hit the top of the elevator. Safing elbow angle.");
			errorCount++;
			if (Math.abs(minAboveAngle.getRadian() - goalState.getElbowAngle().getRadian()) <= Math.abs(maxAboveAngle.getRadian() - goalState.getElbowAngle().getRadian())) {
				goalState.setElbowAngle(minAboveAngle);
				corrCount++;
			} else {
				goalState.setElbowAngle(maxAboveAngle);
				corrCount++;
			}
		}

		//checks if the elevator will go to high
		if (goalState.elevator.height.getFeet() > top.getFeet()) {
			System.out.println("MOTION IMPOSSIBLE -- Elevator will pass maximum height. Setting to maximum height.");
			errorCount++;
			corrCount++;
			goalState.getElevator().setHeight(top);
		}

		//checks if intake will hit crossbar
		if ((throughAbove && gHeight.getFeet() > crossbar.getFeet() - disInside.getFeet() && gHeight.getFeet() < crossbar.getFeet())
				|| throughBelow && gHeight.getFeet() > crossbar.getFeet() && gHeight.getFeet() < crossbar.getFeet() + disInside.getFeet()) {
			System.out.println("MOTION UNSAFE -- Intake will hit crossbar. Setting to default intake position for movement.");
			errorCount++;
			toReturn.add(new SuperStructureState(currentState.elevator, iPosition.CARGO_GRAB, currentState.getHeldPiece()));
		}

		//move to corrected state
		toReturn.add(new SuperStructureState(goalState));
		currentState = goalState;

		System.out.println("MOTION COMPLETED -- " + Integer.valueOf(errorCount) + " error(s) and "
				+ Integer.valueOf(corrCount) + " final correction(s)");
		this.currentPlannedState = currentState;
		return toReturn;
	}

	public SuperStructureState getPlannedState(SuperStructureState goalStateIn, SuperStructureState currentState) {
		this.plan(goalStateIn, currentState);
		return this.currentPlannedState;
	}
}
