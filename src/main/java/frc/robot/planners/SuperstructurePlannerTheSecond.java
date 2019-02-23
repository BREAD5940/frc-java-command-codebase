package frc.robot.planners;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.SuperStructureConstants;
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
public class SuperstructurePlannerTheSecond {

	public SuperstructurePlannerTheSecond() {}
	

	public static final SuperStructureState passThroughState = new SuperStructureState(new ElevatorState(SuperStructureConstants.Elevator.crossbarBottom),
			new RotatingArmState(RoundRotation2d.getDegree(-90)), new RotatingArmState(RoundRotation2d.getDegree(-90)));

	Length minSafeHeight = SuperStructureConstants.Elevator.bottom;
	Length maxSafeHeight = SuperStructureConstants.Elevator.top;

	Length disInside = SuperStructureConstants.Elevator.bottom;

	Translation2d wristPoint;
	Translation2d endPointOut; //perp. to hex and wA
	Translation2d endPointDown; // perp. to hex and wA-90
	Translation2d endPointUp; // perp. to hex and wA+90
	Translation2d carriagePoint;
	Translation2d lowestPoint;
	Translation2d highestPoint;

	boolean throughBelow = false;
	boolean throughAbove = false;
	boolean goingToCrash = false;

	int errorCount; //number of errors in motion
	int corrCount; //number of corrected items in motion
	SuperStructureState currentPlannedState;

	public SuperStructureState plan(SuperStructureState goalIn, SuperStructureState currentIn){
		SuperStructureState goalState = new SuperStructureState(goalIn);

		//checks if its the same
		if (goalState == currentIn){
			System.out.println("MOTION UNNECESSARY -- Goal and current states are the same.");
			this.currentPlannedState = goalState;
			return goalState;
		}

		//checks if it's too high
		if(goalState.getElevatorHeight().getInch() > SuperStructureConstants.Elevator.top.getInch()){
			System.out.println("MOTION IMPOSSIBLE -- Elevator will pass max height. Setting to max.");
			goalState.getElevator().setHeight(SuperStructureConstants.Elevator.top);
		}

		//checks if its' too low
		if(goalState.getElevatorHeight().getInch() < SuperStructureConstants.Elevator.bottom.getInch()){
			System.out.println("MOTION IMPOSSIBLE -- Elevator will pass min height. Setting to min.");
			goalState.getElevator().setHeight(SuperStructureConstants.Elevator.bottom);
		}

		//checks if elbow rotates too high
		if(goalState.getElbow().angle.getDegree() > SuperStructureConstants.Elbow.kElbowMax.getDegree()){
			System.out.println("MOTION IMPOSSIBLE -- Elbow will pass max angle. Setting to max.");
			goalState.getElbow().setAngle(SuperStructureConstants.Elbow.kElbowMax);
		}

		//checks if elbow rotates too low
		if(goalState.getElbow().angle.getDegree() < SuperStructureConstants.Elbow.kElbowMin.getDegree()){
			System.out.println("MOTION IMPOSSIBLE -- Elbow will pass min angle. Setting to min.");
			goalState.getElbow().setAngle(SuperStructureConstants.Elbow.kElbowMin);
		}

		//checks if wrist rotates too high
		if(goalState.getWrist().angle.getDegree() > SuperStructureConstants.Wrist.kWristMax.getDegree()){
			System.out.println("MOTION IMPOSSIBLE -- Wrist will pass max angle. Setting to max.");
			goalState.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMax);
		}

		//checks if wrist rotates too low
		if(goalState.getWrist().angle.getDegree() < SuperStructureConstants.Wrist.kWristMin.getDegree()){
			System.out.println("MOTION IMPOSSIBLE -- Wrist will pass min angle. Setting to min.");
			goalState.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMin);
		}


		//based on the adjusted heights and angles, the cartesian points

		//the point the bottom of the carriage is at
		carriagePoint  = new Translation2d(LengthKt.getInch(0), goalState.getElevatorHeight());

		//the point the center of the wrist joint is at
		wristPoint = new Translation2d(carriagePoint.getX().plus(LengthKt.getInch(
								goalState.getElbow().angle.getCos() * SuperStructureConstants.Elbow.carriageToIntake.getInch())),
					carriagePoint.getY().plus(LengthKt.getInch(
								goalState.getElbow().angle.getSin() * SuperStructureConstants.Elbow.carriageToIntake.getInch())));

		//the point on the intake directly perpendicular to 



		// System.out.println("MOTION COMPLETED -- " + Integer.valueOf(errorCount) + " error(s) and "
		// 		+ Integer.valueOf(corrCount) + " final correction(s)");
		this.currentPlannedState = goalState;
		return goalState;
	}

	public SuperStructureState getPlannedState(SuperStructureState goalStateIn, SuperStructureState currentState) {
		this.plan(goalStateIn, currentState, false);
		return this.currentPlannedState;
	}
}
