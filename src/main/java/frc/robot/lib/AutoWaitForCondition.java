package frc.robot.lib;

import java.util.concurrent.Callable;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

public class AutoWaitForCondition extends SendableCommandBase {

	Callable<Boolean> mCaller; // the command group that called me
	boolean ready = false;

	/**
	 * Wait for a 'super ish' AutoCommandgroup to be ReadyForNext before returning.
	 * @param caller
	 */
	public AutoWaitForCondition(Callable<Boolean> caller) {
		this.mCaller = caller;
	}

	@Override
	public boolean isFinished() {
		boolean isDone;
		try {
			isDone = mCaller.call();
		} catch (Exception e) {
			isDone = true;
		}
		return isDone;
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("===== auto wait for condition command is complete! =====");
	}

}
