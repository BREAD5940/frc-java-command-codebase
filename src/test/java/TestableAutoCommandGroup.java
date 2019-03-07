
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.Logger;

/**
 * basically just a CommandGroup but with the done() method and time tracking.
 */
public class TestableAutoCommandGroup /*extends CommandGroup*/ {

	double start = 0;

	ArrayList<String> mCommandList = new ArrayList<>();

	// @Override
	public synchronized void start() {
		// super.start();
		start = Timer.getFPGATimestamp();
	}

	// @Override
	protected void end() {
		Logger.log("Path ran in " + (Timer.getFPGATimestamp() - start) + " seconds!");
	}

	public TestableAutoCommandGroup() {}

	public boolean done() {
		// return this.isFinished();
		return false;
	}

	public final synchronized void addSequentialLoggable(Command command, double timeout, boolean isReal) {
		// if (isReal)
		// super.addSequential(command, timeout);
		addSequentialLoggable(command, isReal);
	}

	public final synchronized void addSequentialLoggable(Command command, boolean isReal) {
		// if (isReal)
		// super.addSequential(command);
		logCommand(command, "Sequential");
	}

	public final synchronized void addParallelLoggable(Command command, double timeout, boolean isReal) {
		// if (isReal)
		// super.addParallel(command, timeout);
		addParallelLoggable(command, isReal);
	}

	public final synchronized void addParallelLoggable(Command command, boolean isReal) {
		// if (isReal)
		// super.addParallel(command);
		logCommand(command, "Parallel");
	}

	public final synchronized void logCommand(Command command, String mode) {
		var commandName = command.getName();
		var commandSubsystem = command.getSubsystem();
		mCommandList.add(String.format("Command %s added in %s mode and reserves %s", commandName, mode, commandSubsystem));
	}

	public final ArrayList<String> getCommandLog() {
		return mCommandList;
	}

}
