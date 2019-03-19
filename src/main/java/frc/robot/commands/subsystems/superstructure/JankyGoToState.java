package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class JankyGoToState extends CommandGroup {

	public JankyGoToState(Length height, IntakeAngle angles) {
		this(new SuperStructureState(new ElevatorState(height), angles));
	}

	public JankyGoToState(ElevatorState height, IntakeAngle angles) {
		this(new SuperStructureState(height, angles));
	}

	public JankyGoToState(SuperStructureState requ_) {

		requires(SuperStructure.getInstance());

		setInterruptible(false);

		addSequential(new PrintCommand("requested state: " + requ_.toCSV()));

		var safeMove = new ConditionalCommand(new ElevatorThanArm(requ_), new ArmThanElevator(requ_)) {

			@Override
			protected boolean condition() {
				var proximalThreshold = -18;
				var currentState = SuperStructure.getInstance().getCurrentState();
				var startAboveSafe = requ_.getElevatorHeight().getInch() > 25;
				var endAboveSafe = currentState.getElevatorHeight().getInch() > 25;
				var nowOutsideFrame = currentState.getElbowAngle().getDegree() > proximalThreshold;
				var willBeOutsideFrame = requ_.getElbowAngle().getDegree() > proximalThreshold;

				var shouldMoveElevatorFirst = (nowOutsideFrame && !willBeOutsideFrame && !startAboveSafe) || (nowOutsideFrame && willBeOutsideFrame);

				System.out.println("requested state: " + requ_);

				System.out.println(((shouldMoveElevatorFirst) ? "We are moving the elevator first!" : "We are moving the arm first!"));

				return shouldMoveElevatorFirst;

			}
		};

		var choosePath = new ConditionalCommand(new SuperstructureGoToState(requ_), safeMove) {

			@Override
			protected boolean condition() {
				var proximalThreshold = -10;
				var currentState = SuperStructure.getInstance().getCurrentState();
				var nowOutsideFrame = currentState.getElbowAngle().getDegree() > proximalThreshold;
				var willBeOutsideFrame = requ_.getElbowAngle().getDegree() > proximalThreshold;

				var safeToMoveSynced = nowOutsideFrame && willBeOutsideFrame;

				System.out.println("Safe to move synced? " + safeToMoveSynced);

				return safeToMoveSynced;
			}
		};

		addSequential(choosePath);

	}

	public class ElevatorThanArm extends CommandGroup {
		public ElevatorThanArm(SuperStructureState requ) {
			addSequential(new ElevatorMove(requ.getElevator()));
			addSequential(new ArmMove(requ.getAngle()));
		}
	}

	public class ArmThanElevator extends CommandGroup {
		public ArmThanElevator(SuperStructureState requ) {
			addSequential(new ArmMove(requ.getAngle()));
			addSequential(new ElevatorMove(requ.getElevator()));
		}
	}

}
