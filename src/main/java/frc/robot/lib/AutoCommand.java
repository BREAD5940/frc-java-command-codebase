package frc.robot.lib;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import edu.wpi.first.wpilibj.Timer;

/**
 * basically just a CommandGroup but with the done() method and time tracking.
 */
public abstract class AutoCommand extends SendableCommandBase {

	double start = 0;

	// @Override
	// public synchronized void start() {
	// 	super.schedule();
	// 	start = Timer.getFPGATimestamp();
	// }

	@Override
	public void initialize() {
		start = Timer.getFPGATimestamp();
		super.initialize();
	}

	@Override
	public void end(boolean interrupted) {
		Logger.log(super.getName() + " ran in " + (Timer.getFPGATimestamp() - start) + " seconds!");
	}

	public AutoCommand() {}

	public boolean done() {
		return this.isFinished();
	}

}
