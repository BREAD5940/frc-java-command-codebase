//package frc.robot.lib;
//
//import edu.wpi.first.wpilibj.Timer;
//import org.team5940.pantry.exparimental.command.SendableCommandBase;
//
///**
// * basically just a CommandGroup but with the done() method and time tracking.
// */
//public abstract class AutoCommand extends SendableCommandBase {
//
//	double start = 0;
//
//	public synchronized void init() {
//		super.start();
//		start = Timer.getFPGATimestamp();
//	}
//
//	@Override
//	protected void end() {
//		Logger.log(super.getName() + " ran in " + (Timer.getFPGATimestamp() - start) + " seconds!");
//	}
//
//	public AutoCommand() {}
//
//	public boolean done() {
//		return this.isFinished();
//	}
//
//}
