package frc.robot.planners;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.RobotConfig;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
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

	//TODO add values for certain elevator positions (ex. the wrist can be <0 if the elevator is >10)

	//TODO get actual irl angles TODO make the names less horrible

	static final Length minUnCrashHeight = LengthKt.getInch(5); //min elevator height + how much intake is below the bottom of the elevator

	static final Length crossbarMinHeight = LengthKt.getInch(20);
	static final Length crossbarMaxHeight = LengthKt.getInch(24);

	static final Length maxHeight = RobotConfig.elevator.elevator_maximum_height;

	boolean intakeCrashable = false; //intake capable of hitting the ground
	boolean intakeAtRisk = false; //intake at risk of hitting the crossbar
	int errorCount; //number of errors in motion
	int corrCount; //number of corrected items in motion
	SuperStructureState currentPlannedState;

	public boolean checkValidState(SuperStructureState reqState) { //what is this supposed to do? does it just check if the path is possible w/o correction?
		return true; // FIXME to check if it's valid. Maybe call planner.plan? idk
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
		SuperStructureState goalState = goalStateIn;
		errorCount = corrCount = 0;
		boolean defAngle = iPosition.presets.contains(goalState.getAngle());

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

		if (!defAngle) {
			System.out.println("MOTION UNSAFE -- Wrist position is wildcard. Setting to default position for movement.");
			errorCount++;
			if (currentState.getHeldPiece() == HeldPiece.HATCH) {
				//TODO change this so it only happens if the intake will ACTUALLY pass through the elevator
				System.out.println("MOTION UNSAFE -- Cannot move wrist to wildcard position while holding hatch. Aborting wrist movement.");
				errorCount++;
				corrCount++;
				goalState.setAngle(iPosition.CARGO_GRAB);
			} else {
				toReturn.add(new SuperStructureState(currentState.elevator, iPosition.CARGO_GRAB, currentState.getHeldPiece()));
				intakeAtRisk = false;
				intakeCrashable = false;
			}
		} else {
			// Checks if the intake will ever be inside the elevator
			if ((currentState.getAngle() == iPosition.HATCH) || (goalState.getAngle() == iPosition.HATCH)) {
				intakeAtRisk = true;
			}

			//checks if the intake will tilt/is tilted below the bottom of the elevator
			if ((goalState.getAngle() == iPosition.CARGO_DOWN) || (currentState.getAngle() == iPosition.CARGO_DOWN)) { // FIXME so this will only check for exact equivilency, not for a "less than" condition. Same with all the currentStates I think
				intakeCrashable = true;
			}
		}

		//checks if the elevator will go to high
		if (goalState.elevator.height.getValue() > maxHeight.getValue()) {
			System.out.println("MOTION IMPOSSIBLE -- Elevator will pass maximum height. Setting to maximum height.");
			errorCount++;
			corrCount++;
			goalState.getElevator().setHeight(maxHeight);
		}

		//checks if the elevator will move past the crossbar
		if (intakeAtRisk && (goalState.getElevatorHeight().getValue() >= crossbarMaxHeight.getValue() && currentState.getElevatorHeight().getValue() <= crossbarMinHeight.getValue())
				|| (goalState.getElevatorHeight().getValue() <= crossbarMinHeight.getValue() && currentState.getElevatorHeight().getValue() >= crossbarMaxHeight.getValue())) {
			System.out.println("MOTION UNSAFE -- Intake will hit crossbar. Setting to default intake position for movement.");
			errorCount++;
			toReturn.add(new SuperStructureState(currentState.elevator, iPosition.CARGO_GRAB, currentState.getHeldPiece())); //Keeps intake outside the elevator so it doesn't hit the crossbar
		} else {
			intakeAtRisk = false;
		}

		//checks if the elevator will move low enough to crash the intake
		if (goalState.getElevatorHeight().getValue() <= minUnCrashHeight.getValue() && intakeCrashable) {
			System.out.println("MOTION UNSAFE -- Intake will hit ground. Setting to default intake position.");
			errorCount++;
			corrCount++;
			goalState.setAngle(iPosition.CARGO_GRAB);
		} else {
			intakeCrashable = false;
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
