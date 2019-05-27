package frc.robot.lib;

import java.util.function.Supplier;

import org.team5940.pantry.exparimental.command.Command;
import org.team5940.pantry.exparimental.command.ParallelCommandGroup;

public class OLDParallelRaceGroup extends ParallelCommandGroup {

	Supplier<Boolean> m_condition;

	public OLDParallelRaceGroup(Supplier<Boolean> condition, Command... commands) {

		m_condition = condition;

		super.addCommands(commands);

		//		for (Command command : commands) {
		//			super.addParallel(command);
		//		}

	}

	@Override
	public boolean isFinished() {
		return super.isFinished() || m_condition.get().booleanValue();
	}

}
