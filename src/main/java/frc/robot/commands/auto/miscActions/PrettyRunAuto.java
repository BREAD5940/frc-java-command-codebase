package frc.robot.commands.auto.miscActions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.routines.AutoCommandGroup;
import frc.robot.lib.statemachines.AutoMotionStateMachine;

public class PrettyRunAuto extends Command {

	protected final AutoMotionStateMachine machine;
	protected boolean onlyPreset;
	protected AutoMotion createdMotion;
	protected AutoCommandGroup mainCommand;
	protected AutoCommandGroup ssCommand;
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
	protected void initialize() {
		ssCommand.start();
	}

	@Override
	protected void execute() {
		System.out.printf("Done? %b\n", ssCommand.done());

		if (ssCommand.done() && !mainBegun && !onlyPreset) {
			createdMotion.getBigCommandGroup().start();
			mainBegun = true;
		}
	}

	@Override
	protected boolean isFinished() {
		if (onlyPreset) {
			return ssCommand.done();
		} else {
			return mainBegun && mainCommand.done();
		}
	}
}
