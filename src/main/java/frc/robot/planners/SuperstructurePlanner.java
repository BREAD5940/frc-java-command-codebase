package frc.robot.planners;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.RobotConfig;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
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

	public static final Length bottom = LengthKt.getInch(RobotConfig.elevator.elevator_minimum_height.getInch());
	public static final Length top = RobotConfig.elevator.elevator_maximum_height;
	static final Length crossbarBottom = LengthKt.getInch(35); //FIXME verify
	static final Length crossbarWidth = LengthKt.getInch(4); //FIXME verify
	static final Length carriageToIntake = LengthKt.getInch(12); //FIXME verify
	static final Length intakeDiameter = LengthKt.getInch(19);

	public static final RoundRotation2d overallMaxElbow = RoundRotation2d.getDegree(15); //FIXME actual numbers might be nice
	public static final RoundRotation2d overallMinElbow = RoundRotation2d.getDegree(-190); //FIXME ^^
	public static final RoundRotation2d overallMaxWrist = RoundRotation2d.getDegree(180); //FIXME ^^
	public static final RoundRotation2d overallMinWrist = RoundRotation2d.getDegree(-180); //FIXME ^^

	public static final SuperStructureState passThroughState = new SuperStructureState(new ElevatorState(crossbarBottom),
			new RotatingArmState(RoundRotation2d.getDegree(-90)), new RotatingArmState(RoundRotation2d.getDegree(-90)));

	Length minSafeHeight = bottom;
	Length maxSafeHeight = top;

	Length disInside = bottom;

	Translation2d wristPoint;
	Translation2d endPoint;
	Translation2d carriagePoint;

	boolean throughBelow = false;
	boolean throughAbove = false;
	boolean goingToCrash = false;

	int errorCount; //number of errors in motion
	int corrCount; //number of corrected items in motion
	SuperStructureState currentPlannedState;

	@Deprecated //we shouldn't use this
	public boolean checkValidState(SuperStructureState reqState) { //what is this supposed to do? does it just check if the path is possible w/o correction?
		// TODO is this what we actuall want this to looke like?
		ArrayList<SuperStructureState> plannedPath = this.plan(reqState, this.currentPlannedState);
		return (plannedPath.get(plannedPath.size() - 1).isEqualTo(plannedPath.get(0)) && plannedPath.size() == 1);
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
		ArrayList<SuperStructureState> toReturn = new ArrayList<SuperStructureState>();// = new List<SuperStructureState>();
		SuperStructureState goalState = new SuperStructureState(goalStateIn);
		errorCount = corrCount = 0;

		//checks if the elevator will go to high
		if (goalState.elevator.height.getInch() > top.getInch()) {
			System.out.println("MOTION IMPOSSIBLE -- Elevator will pass maximum height. Setting to maximum height.");
			errorCount++;
			corrCount++;
			goalState.getElevator().setHeight(top);
		} else if (goalState.elevator.height.getInch() < bottom.getInch()) {
			System.out.println("MOTION IMPOSSIBLE -- Elevator will attempt to smash directly through the bottom of the robot. Setting to minimum height.");
			errorCount++;
			corrCount++;
			goalState.getElevator().setHeight(bottom);
		}

		//TODO do we need these angle checks?
		if (goalState.getElbowAngle().getDegree() > overallMaxElbow.getDegree()) {
			System.out.println("MOTION IMPOSSIBLE -- Elbow passes hardstop. Setting to maximum.");
			errorCount++;
			corrCount++;
			goalState.getElbow().setAngle(overallMaxElbow);
		} else if (goalState.getElbowAngle().getDegree() < overallMinElbow.getDegree()) {
			System.out.println("MOTION IMPOSSIBLE -- Elbow passes hardstop. Setting to minimum.");
			errorCount++;
			corrCount++;
			goalState.getElbow().setAngle(overallMinElbow);
		}

		if (goalState.getWrist().angle.getDegree() > overallMaxWrist.getDegree()) {
			System.out.println("MOTION IMPOSSIBLE -- Wrist passes hardstop. Setting to maximum.");
			errorCount++;
			corrCount++;
			goalState.getWrist().setAngle(overallMaxWrist);
		} else if (goalState.getWrist().angle.getDegree() < overallMinWrist.getDegree()) {
			System.out.println("MOTION IMPOSSIBLE -- Wrist passes hardstop. Setting to minimum.");
			errorCount++;
			corrCount++;
			goalState.getWrist().setAngle(overallMinWrist);
		}

		carriagePoint = new Translation2d(LengthKt.getInch(0), goalState.getElevatorHeight());
		wristPoint = new Translation2d(LengthKt.getInch(Math.cos(goalState.getElbowAngle().getRadian()) * carriageToIntake.getInch()),
				carriagePoint.getY().plus(LengthKt.getInch(Math.sin(goalState.getAngle().getElbow().angle.getRadian()) * carriageToIntake.getInch())));
		endPoint = new Translation2d(wristPoint.getX().plus(LengthKt.getInch(Math.cos(goalState.getWrist().angle.getRadian()) * intakeDiameter.getInch())),
				wristPoint.getY().plus(LengthKt.getInch(Math.sin(goalState.getAngle().getWrist().angle.getRadian()) * intakeDiameter.getInch())));

		if ((wristPoint.getX().getInch() < 0 && Math.cos(currentState.getElbowAngle().getRadian()) * carriageToIntake.getInch() > 0)
				|| (wristPoint.getX().getInch() > 0 && Math.cos(currentState.getElbowAngle().getRadian()) * carriageToIntake.getInch() < 0)) { //FIXME this probably needs another thingy
			//if it passes through the elevator between current and goal
			System.out.println("MOTION UNSAFE -- Can't pass through the elevator at points other than the pass-through point. Adding to path.");
			errorCount++;
			toReturn.add(passThroughState);
		}

		//le booleans
		//FIXME this currently doesn't check if the 'forearm' is in the elevator'
		throughBelow = endPoint.getY().getInch() < goalState.getElevatorHeight().getInch() && ((wristPoint.getX().getInch() > 0 && endPoint.getX().getInch() < 0) || (endPoint.getX().getInch() > 0 && wristPoint.getX().getInch() < 0));

		throughAbove = endPoint.getY().getInch() > goalState.getElevatorHeight().getInch() && ((wristPoint.getX().getInch() > 0 && endPoint.getX().getInch() < 0) || (endPoint.getX().getInch() > 0 && wristPoint.getX().getInch() < 0));

		if (throughAbove || throughBelow) {
			disInside = LengthKt.getInch(endPoint.getX().getInch() - (endPoint.getY().getInch() * ((endPoint.getX().getInch() - wristPoint.getX().getInch()) / (endPoint.getY().getInch() - wristPoint.getY().getInch()))));
		} else {
			disInside = goalState.getElevatorHeight();
		}

		goingToCrash = (throughBelow && goalState.getElevator().getHeight().getFeet() < disInside.getFeet())
				|| (throughAbove && goalState.getElevator().getHeight().getFeet() + disInside.getFeet() > top.getFeet());

		if (goalState == currentState) {
			System.out.println("MOTION UNNECESSARY -- Goal and current states are same. Exiting planner.");
			this.currentPlannedState = goalState;
			return new ArrayList<SuperStructureState>(Arrays.asList(goalState));
		}

		if (goalState.getHeldPiece() != currentState.getHeldPiece()) {
			System.out.println("MOTION IMPOSSIBLE -- Superstructure motion cannot change heldPiece. Resolving error.");
			errorCount++;
			corrCount++;
			goalState.setHeldPiece(currentState.getHeldPiece());
		}

		if (endPoint.getY().getFeet() < bottom.getFeet() + disInside.getFeet()) {
			System.out.println("MOTION IMPOSSIBLE -- Intake will hit literally all of the electronics. Safing elbow angle.");
			errorCount++;
			//finds the necessary angle of the elbow to safe the intake
			double cTheta = Math.asin(carriageToIntake.getInch() * ((bottom.getInch() + disInside.getInch()) - goalState.getElevatorHeight().getInch() -
					Math.sin(goalState.getAngle().getWrist().angle.getRadian())));
			//checks if the adjustment is too big
			if (cTheta < goalState.getAngle().getElbow().angle.getRadian() && cTheta != 0) {
				System.out.println("MOTION UNSAFE -- Angle cannot be safed with only elbow. Safing wrist.");
				//sets the elbow to the max allowed adjustment
				goalState.setElbowAngle(RoundRotation2d.getRadian(0));
				//finds the necessary wrist angle to finish safing the intake
				cTheta = Math.asin(intakeDiameter.getInch() * ((bottom.getInch() + disInside.getInch()) - goalState.getElevatorHeight().getInch() -
						Math.sin(goalState.getAngle().getElbow().angle.getRadian())));
				//sets the wrist to that angle
				goalState.getAngle().getWrist().setAngle(RoundRotation2d.getRadian(cTheta));
				corrCount += 2;
			} else {
				goalState.setElbowAngle(RoundRotation2d.getRadian(cTheta));
				corrCount++;
			}
		}
		// TODO make sure commenting this out doesn't break things
		// if (throughAbove && endPoint.getY().getFeet() > top.getFeet() - disInside.getFeet()) {
		// 	System.out.println("MOTION UNSAFE -- Intake will hit the top of the elevator. Safing elbow angle.");
		// 	errorCount++;
		// 	if (Math.abs(minAboveAngle.getRadian() - goalState.getElbowAngle().getRadian()) <= Math.abs(maxAboveAngle.getRadian() - goalState.getElbowAngle().getRadian())) {
		// 		goalState.setElbowAngle(RoundRotation2d.fromRotation2d(minAboveAngle));
		// 		corrCount++;
		// 	} else {
		// 		goalState.setElbowAngle(RoundRotation2d.fromRotation2d(maxAboveAngle));
		// 		corrCount++;
		// 	}
		// }

		//checks if intake will hit crossbarBottom
		if ((throughAbove && endPoint.getY().getFeet() > crossbarBottom.getFeet() - disInside.getFeet() && endPoint.getY().getFeet() < crossbarBottom.getFeet())
				|| throughBelow && endPoint.getY().getFeet() > crossbarBottom.getFeet() && endPoint.getY().getFeet() < crossbarBottom.getFeet() + disInside.getFeet()) {
			System.out.println("MOTION UNSAFE -- Intake will hit crossbarBottom. Setting to default intake position for movement.");
			errorCount++;
			toReturn.add(new SuperStructureState(currentState.elevator, iPosition.CARGO_GRAB, currentState.getHeldPiece()));
		}

		//move to corrected state
		toReturn.add(new SuperStructureState(goalState));
		currentState = goalState;

		// System.out.println("MOTION COMPLETED -- " + Integer.valueOf(errorCount) + " error(s) and "
		// 		+ Integer.valueOf(corrCount) + " final correction(s)");
		this.currentPlannedState = currentState;
		return toReturn;
	}

	public SuperStructureState getPlannedState(SuperStructureState goalStateIn, SuperStructureState currentState) {
		this.plan(goalStateIn, currentState);
		return this.currentPlannedState;
	}
}
