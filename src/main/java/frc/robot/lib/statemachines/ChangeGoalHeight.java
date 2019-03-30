package frc.robot.lib.statemachines;

import org.team5940.pantry.experimental.command.SendableCommandBase;
import frc.robot.Robot;

/**
 * is literally a toggle
 * 
 * @author jocleyn McHugo
 */
public class ChangeGoalHeight extends SendableCommandBase {
	private boolean up = true;

	public ChangeGoalHeight(boolean up) {
		this.up = up;
	}

	@Override
	protected void initialize() {
		if (up) {
			Robot.autoState.goalHeightUp();
		} else {
			Robot.autoState.goalHeightDown();
		}
	}

	@Override
	protected boolean isFinished() {
		return true;
	}
}
