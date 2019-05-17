package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ElevatorMove extends Command {

	private final ElevatorState mGoal;

	/**
	 * Move
	 * @param goal the goal state of the elevator
	 */
	public ElevatorMove(ElevatorState goal) {
		this(goal, "ElevatorMove to " + goal.toString());
	}

	public ElevatorMove(Length goal) {
		this(new ElevatorState(goal));
	}

	public ElevatorMove(ElevatorState goal, String name) {
		this.mGoal = goal;
		requires(SuperStructure.getElevator());
		setName(name);
	}

	@Override
	protected void initialize() {}

	@Override
	protected void execute() {

//		System.out.println("moving to " + mGoal);

		// double elevatorPercentVbusGravity = Elevator.getVoltage(SuperStructure.getInstance().updateState()) / 12;
		SuperStructure.getElevator().setPositionSetpoint(mGoal);

	}

	@Override
	protected boolean isFinished() {
		var toReturn = Math.abs(SuperStructure.getElevator().getHeight().getInch() - mGoal.height.getInch()) <= 1;
//		System.out.println("ELEVATOR DONE? " + toReturn);
		return toReturn;
	}

}
