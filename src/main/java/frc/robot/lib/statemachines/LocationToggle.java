package frc.robot.lib.statemachines;

import org.team5940.pantry.experimental.command.SendableCommandBase;

import frc.robot.Robot;
import frc.robot.lib.statemachines.AutoMotionStateMachine.GoalLocation;

/**
 * is literally a toggle
 * 
 * @author jocleyn McHugo
 */
public class LocationToggle extends SendableCommandBase {
	public LocationToggle() {}

	@Override
	public void initialize() {
		Robot.autoState.setGoalLocation(GoalLocation.CARGO_SHIP);
	}

	@Override
	public void end(boolean interrupted) {
		Robot.autoState.setGoalLocation(GoalLocation.ROCKET);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
