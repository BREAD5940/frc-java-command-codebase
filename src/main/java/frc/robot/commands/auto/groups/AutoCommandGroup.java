package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.lib.Logger;

/**
 * basically just a CommandGroup but with the done() method
 */
public class AutoCommandGroup extends CommandGroup {

	double start = 0;

	@Override
	public synchronized void start() {
		super.start();
		start = Timer.getFPGATimestamp();
	}

	@Override
	protected void end() {
    Logger.log("Path ran in " + (Timer.getFPGATimestamp() - start) + " seconds!");
	}

	public AutoCommandGroup() {}

	public boolean done() {
		return this.isFinished();
	}

}
