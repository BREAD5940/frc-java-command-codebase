package frc.robot.commands.auto;

import org.team5940.pantry.exparimental.command.Command;
import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.lib.statemachines.AutoMotionStateMachine;

public class PrettyRunAuto extends SendableCommandBase {

	protected final AutoMotionStateMachine machine;
	protected boolean onlyPreset;
	protected AutoMotion createdMotion;
	protected Command mainCommand;
	protected Command ssCommand;
	protected boolean mainBegun = false;

	public PrettyRunAuto(AutoMotionStateMachine machine, boolean onlyPreset) {
		//plz no require thing
		this.machine = machine;
		this.onlyPreset = onlyPreset;
		createdMotion = new AutoMotion(machine.getGoalHeight(), machine.getGoalType(), false);
		ssCommand = createdMotion.getPrepCommand();
		mainCommand = createdMotion.getBigCommandGroup();
	}

	@Override
	public void initialize() {
		ssCommand.schedule();
	}

	@Override
	public void execute() {
		System.out.printf("Done? %b\n", ssCommand.isFinished());

		if (ssCommand.isFinished() && !mainBegun && !onlyPreset) {
			createdMotion.getBigCommandGroup().schedule();
			mainBegun = true;
		}
	}

	@Override
	public boolean isFinished() {
		if (onlyPreset) {
			return ssCommand.isFinished();
		} else {
			return mainBegun && mainCommand.isFinished();
		}
	}
}
