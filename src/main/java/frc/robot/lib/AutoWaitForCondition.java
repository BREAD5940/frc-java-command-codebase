package frc.robot.lib;

import java.util.concurrent.Callable;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.command.Command;

public class AutoWaitForCondition extends Command {

	BooleanSupplier mCaller; // the command group that called me
	boolean ready = false;

	/**
	 * Wait for a 'super ish' AutoCommandgroup to be ReadyForNext before returning.
	 */
	public AutoWaitForCondition(BooleanSupplier caller) {
		this.mCaller = caller;
	}

	@Override
	protected boolean isFinished() {
		boolean isDone;
		try {
			isDone = mCaller.getAsBoolean();
		} catch (Exception e) {
			isDone = true;
		}
		return isDone;
	}

	@Override
	protected void end() {
		System.out.println("===== auto wait for condition command is complete! =====");
	}

}
