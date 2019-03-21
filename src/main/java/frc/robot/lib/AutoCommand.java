package frc.robot.lib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * basically just a CommandGroup but with the done() method and time tracking.
 */
public abstract class AutoCommand extends Command {

	double start = 0;

	@Override
	public synchronized void start() {
		super.start();
		start = Timer.getFPGATimestamp();
	}

	@Override
	protected void end() {
		Logger.log(super.getName() + " ran in " + (Timer.getFPGATimestamp() - start) + " seconds!");
	}

	public AutoCommand() {}

	public boolean done() {
		return this.isFinished();
	}

}
