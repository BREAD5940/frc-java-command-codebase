//package frc.robot.commands.auto;
//
//import org.team5940.pantry.exparimental.command.InstantCommand;
//
//
///**
// * Add your docs here.
// */
//public class InstantRunnable extends InstantCommand {
//
//	Runnable toRun;
//
//	public InstantRunnable(Runnable thing, boolean runWhenDisabled) {
//		super();
//		setRunWhenDisabled(runWhenDisabled);
//		this.toRun = thing;
//	}
//
//	@Override
//	public boolean runsWhenDisabled() {
//		return false;
//	}
//
//	// Called once when the command executes
//	@Override
//	public void initialize() {
//		try {
//			toRun.run();
//		} catch (Exception e) {
//			//TODO: handle exception
//		}
//	}
//
//}
