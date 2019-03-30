package frc.robot.lib;

import java.util.concurrent.Callable;

import org.team5940.pantry.experimental.command.SendableCommandBase;

public class AutoWaitForCondition extends SendableCommandBase {

	Callable<Boolean> mCaller; // the command group that called me
	boolean ready = false;

	/**
	 * Wait for a 'super ish' AutoCommandgroup to be ReadyForNext before returning.
	 */
	public AutoWaitForCondition(Callable<Boolean> caller) {
		this.mCaller = caller;
	}

	@Override
	protected boolean isFinished() {
		boolean isDone;
		try {
			isDone = mCaller.call().booleanValue();
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
