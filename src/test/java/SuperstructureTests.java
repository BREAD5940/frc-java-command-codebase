import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.junit.jupiter.api.Test;

import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.planners.*;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class SuperstructureTests {

	// @Test
	// public void testWrists() {
	// 	SuperstructurePlanner planner = new SuperstructurePlanner();
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
	// 	SuperstructurePlanner planner = new SuperstructurePlanner();
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
	// 	SuperstructurePlanner planner = new SuperstructurePlanner();
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
	public void testElevatorVoltage() {
		Mass kCarriageMass = MassKt.getLb(9.3);
		Mass kInnerStageMass = MassKt.getLb(6.5);

		Mass kHatchMass = MassKt.getLb(2.4);
		Mass kCargoMass = MassKt.getLb(1);

		double kLowGearForcePerVolt = (512d / 12d) * 1.5;
		double kHighGearForcePerVolt = (1500d / 12d);


		ArrayList<SuperStructureState> states = new ArrayList<SuperStructureState>(Arrays.asList(
				new SuperStructureState(new ElevatorState(LengthKt.getInch(5)), new RotatingArmState(), new RotatingArmState()),
		new SuperStructureState(new ElevatorState(LengthKt.getInch(30)), new RotatingArmState(), new RotatingArmState()),
		new SuperStructureState(new ElevatorState(LengthKt.getInch(50)), new RotatingArmState(), new RotatingArmState()),
		new SuperStructureState(new ElevatorState(LengthKt.getInch(70)), new RotatingArmState(), new RotatingArmState())
		));
		ArrayList<Double> correctVolts = new ArrayList<Double>(Arrays.asList(
				((Elevator.kCarriageMass.getKilogram()) * 9.81) / Elevator.KHighGearForcePerVolt,
				((Elevator.kCarriageMass.getKilogram()) * 9.81) / Elevator.KHighGearForcePerVolt,
				((Elevator.kCarriageMass.getKilogram() + Elevator.kInnerStageMass.getKilogram()) * 9.81) / Elevator.KHighGearForcePerVolt,
				((Elevator.kCarriageMass.getKilogram() + Elevator.kInnerStageMass.getKilogram()) * 9.81) / Elevator.KHighGearForcePerVolt

				));


		for (int i = 0; i < states.size(); i++) {
			System.out.printf("Index: %d   First: %f   Second: %f\n", i, 
				Elevator.getVoltage(states.get(i)), correctVolts.get(i));
			assertEquals(Elevator.getVoltage(states.get(i)), (double) correctVolts.get(i), 0.1);
		}
	}
}
