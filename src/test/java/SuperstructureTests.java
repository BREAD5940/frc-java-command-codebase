import static org.junit.jupiter.api.Assertions.assertEquals;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.junit.jupiter.api.Test;

import frc.robot.lib.Logger;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.planners.SuperstructureMotion;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class SuperstructureTests extends Testable {

	// @Test
	// public void testWrists() {
	// 	SuperstructurePlannerOLD planner = new SuperstructurePlannerOLD();
	// 	SuperStructureState currentState = new SuperStructureState(new ElevatorState(), iPosition.CARGO_GRAB, HeldPiece.NONE);
	// 	ArrayList<SuperStructureState> goalStates = new ArrayList<SuperStructureState>(Arrays.asList(
	// 			new SuperStructureState(new ElevatorState(), iPosition.CARGO_GRAB, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(), iPosition.HATCH, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(), iPosition.CARGO_DOWN, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(),
	// 					new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(10)), new RotatingArmState(Rotation2dKt.getDegree(10))), HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(), iPosition.CARGO_GRAB, HeldPiece.CARGO)));
	// 	ArrayList<SuperStructureState> correctEndStates = new ArrayList<SuperStructureState>(Arrays.asList(
	// 			new SuperStructureState(new ElevatorState(), iPosition.CARGO_GRAB, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(), iPosition.HATCH, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(), iPosition.CARGO_GRAB, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(),
	// 					new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(10)), new RotatingArmState(Rotation2dKt.getDegree(10))), HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(), iPosition.CARGO_GRAB, HeldPiece.NONE)));
	// 	ArrayList<SuperStructureState> resultingStates = new ArrayList<SuperStructureState>();

	// 	for (int i = 0; i < goalStates.size(); i++) {
	// 		resultingStates.add(i, planner.getPlannedState(goalStates.get(i), currentState));
	// 		System.out.print(Integer.valueOf(i) + ": ");
	// 		System.out.print(correctEndStates.get(i).toString());
	// 		System.out.print(", ");
	// 		System.out.println(resultingStates.get(i).toString());
	// 		System.out.println("Correct state and resulting state are equal: " + correctEndStates.get(i).isEqualTo(resultingStates.get(i)));
	// 		if (!correctEndStates.get(i).isEqualTo(resultingStates.get(i))) {
	// 			throw new AssertionError("Expected " + correctEndStates.get(i).toString() + ", got " + resultingStates.get(i).toString());
	// 		}
	// 	}
	// 	System.out.println();
	// }

	// @Test
	// public void testElevator() {
	// 	SuperstructurePlannerOLD planner = new SuperstructurePlannerOLD();
	// 	SuperStructureState currentState = new SuperStructureState(new ElevatorState(true), iPosition.CARGO_GRAB, HeldPiece.NONE);
	// 	ArrayList<SuperStructureState> goalStates = new ArrayList<SuperStructureState>(Arrays.asList(
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(10), true), iPosition.CARGO_GRAB, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(30), true), iPosition.CARGO_GRAB, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(80), true), iPosition.CARGO_GRAB, HeldPiece.NONE)));
	// 	ArrayList<SuperStructureState> correctEndStates = new ArrayList<SuperStructureState>(Arrays.asList(
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(10), true), iPosition.CARGO_GRAB, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(30), true), iPosition.CARGO_GRAB, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(70), true), iPosition.CARGO_GRAB, HeldPiece.NONE)));
	// 	ArrayList<SuperStructureState> resultingStates = new ArrayList<SuperStructureState>();

	// 	for (int i = 0; i < goalStates.size(); i++) {
	// 		resultingStates.add(i, planner.getPlannedState(goalStates.get(i), currentState));
	// 		System.out.print(Integer.valueOf(i) + ": ");
	// 		System.out.print(correctEndStates.get(i).toString());
	// 		System.out.print(", ");
	// 		System.out.println(resultingStates.get(i).toString());
	// 		if (!correctEndStates.get(i).isEqualTo(resultingStates.get(i))) {
	// 			throw new AssertionError("Expected " + correctEndStates.get(i).toString() + ", got " + resultingStates.get(i).toString());
	// 		}
	// 	}
	// 	System.out.println();
	// }

	// @Test
	// public void bigScaryComboTests() {
	// 	SuperstructurePlannerOLD planner = new SuperstructurePlannerOLD();
	// 	SuperStructureState currentState = new SuperStructureState(new ElevatorState(), iPosition.CARGO_GRAB, HeldPiece.NONE);
	// 	ArrayList<SuperStructureState> goalStates = new ArrayList<SuperStructureState>(Arrays.asList(
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(10)), iPosition.HATCH, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(30)), iPosition.HATCH, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(30)),
	// 					new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(10)), new RotatingArmState(Rotation2dKt.getDegree(10))), HeldPiece.NONE)));
	// 	ArrayList<SuperStructureState> correctEndStates = new ArrayList<SuperStructureState>(Arrays.asList(
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(10)), iPosition.HATCH, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(30)), iPosition.HATCH, HeldPiece.NONE),
	// 			new SuperStructureState(new ElevatorState(LengthKt.getInch(30)),
	// 					new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(10)), new RotatingArmState(Rotation2dKt.getDegree(10))), HeldPiece.NONE)));
	// 	ArrayList<SuperStructureState> resultingStates = new ArrayList<SuperStructureState>();

	// 	for (int i = 0; i < goalStates.size(); i++) {
	// 		resultingStates.add(i, planner.getPlannedState(goalStates.get(i), currentState));

	// 		if (!correctEndStates.get(i).isEqualTo(resultingStates.get(i))) {
	// 			throw new AssertionError("Expected " + correctEndStates.get(i).toString() + ", got " + resultingStates.get(i).toString());
	// 		}
	// 	}
	// 	System.out.println();
	// }

	@Test
	public void safingTest() {

		double currentDTVelocity, lastSP = 50, lastLastSP = 50;
		double[] lVels = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 12, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5,
				12.5, 12.5, 11, 11, 11, 12, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0};

		double[][] toPrint = new double[40][3];
		double reqSetHeight = Math.abs(50);
		double currentSetHeight;

		for (int count = 0; count < 39; count++) {
			currentDTVelocity = Math.abs((lVels[count] + lVels[count]) / 2);
			currentSetHeight = reqSetHeight;

			if (currentDTVelocity > 5) {
				currentSetHeight = 0.310544 * Math.pow(currentDTVelocity, 2) - 11.7656 * currentDTVelocity + 119.868;
				if (currentSetHeight > reqSetHeight) {
					currentSetHeight = reqSetHeight;
				}
			}

			toPrint[count][0] = count;
			toPrint[count][1] = (currentSetHeight + lastSP + lastLastSP) / 3;
			toPrint[count][2] = currentDTVelocity;

			lastLastSP = lastSP;
			lastSP = currentSetHeight;
		}

		writeToCSV("src/test/java/safingTestOut.csv", toPrint);
	}

	@Test
	public void superstructureMotionTest() {

		SuperStructureState Start1 = new SuperStructureState(new ElevatorState(LengthKt.getInch(5)),
				new IntakeAngle(new RotatingArmState(RoundRotation2d.getDegree(0)), new RotatingArmState(RoundRotation2d.getDegree(0))));

		SuperStructureState Goal1 = new SuperStructureState(new ElevatorState(LengthKt.getInch(10)),
				new IntakeAngle(new RotatingArmState(RoundRotation2d.getDegree(0)), new RotatingArmState(RoundRotation2d.getDegree(0))));

		testableSSMotion.getInstance().plan(Goal1, Start1);

		Start1 = iPosition.HATCH_GRAB_INSIDE;
		Goal1 = new SuperStructureState(new ElevatorState(LengthKt.getInch(3.5)), iPosition.CARGO_GRAB);
		Logger.log("======== testing hatch grab to cargo pickup ========");
		testableSSMotion.getInstance().plan(Goal1, Start1);

		// var planned = testableSSMotion.getInstance().getQueue();

		// for(Command c : planned.)

	}

	@Test
	public void testMotion() {

		//FIXME Goal1 and Start1 are undefined
		// SuperstructureMotion.getInstance().plan(Goal1, Start1);

		System.out.println(SuperstructureMotion.getInstance().getQueue());

	}

	@Test
	public void rotation2dTest() {
		assertEquals(360, RoundRotation2d.getDegree(360).getDegree(), 0.1);
	}

}
