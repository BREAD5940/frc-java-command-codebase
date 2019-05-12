//package frc.robot.lib;
//
//import java.util.function.Supplier;
//
////import org.team5940.pantry.exparimental.command.SendableCommandBase;
////import edu.wpi.first.wpilibj.command.CommandGroup;
//
//public class ParallelRaceGroup extends SendableCommandBaseGroup {
//
//	Supplier<Boolean> m_condition;
//
//	public ParallelRaceGroup(Supplier<Boolean> condition, Command... commands) {
//
//		m_condition = condition;
//
//		for (Command command : commands) {
//			super.addParallel(command);
//		}
//
//	}
//
//	@Override
//	public boolean isFinished() {
//		return super.isFinished() || m_condition.get().booleanValue();
//	}
//
//}
