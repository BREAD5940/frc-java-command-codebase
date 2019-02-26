package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ElevatorMove extends Command {

	private final ElevatorState mGoal;

	public ElevatorMove(ElevatorState goal) {
		this.mGoal = goal;
		requires(SuperStructure.getElevator());
	}

	@Override
	protected void execute() {
		// como se dice "how should we do dis because elevator gravity feed forward sucks" en espa~nol?
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

}
