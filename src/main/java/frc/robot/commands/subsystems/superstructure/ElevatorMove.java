package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ElevatorMove extends Command {

	private final ElevatorState mGoal;

	public ElevatorMove(ElevatorState goal) {
		this.mGoal = goal;
		requires(SuperStructure.getElevator());
	}

	@Override
	protected void initialize() {}

	@Override
	protected void execute() {
		// como se dice "how should we do dis because elevator gravity feed forward sucks" en espa~nol?\
		// yo no sé, pero lo haré de todos modos

		double elevatorPercentVbusGravity = Elevator.getVoltage(SuperStructure.getInstance().updateState()) / 12;
		SuperStructure.getElevator().setPositionArbitraryFeedForward(mGoal.height, elevatorPercentVbusGravity);

	}

	@Override
	protected boolean isFinished() {
		return Math.abs(SuperStructure.getElevator().getFeet() - mGoal.height.getFeet()) <= 3;
	}

}