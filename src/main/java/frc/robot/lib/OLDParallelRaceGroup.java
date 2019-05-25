package frc.robot.lib;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class OLDParallelRaceGroup extends CommandGroup {

	Supplier<Boolean> m_condition;

	public OLDParallelRaceGroup(Supplier<Boolean> condition, Command... commands) {

		m_condition = condition;

		for (Command command : commands) {
			super.addParallel(command);
		}

	}

	@Override
	protected boolean isFinished() {
		return super.isFinished() || m_condition.get().booleanValue();
	}

}
