package frc.robot.lib.statemachines;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.statemachines.AutoMotionStateMachine.MotionType;

/**
 * is literally a toggle
 * 
 * @author jocleyn McHugo
 */
public class PickupToggle extends Command {
	public PickupToggle() {}

	@Override
	protected void initialize() {
		Robot.autoState.setMotionType(MotionType.PICKUP);
	}

	@Override
	protected void interrupted() {
		Robot.autoState.setMotionType(MotionType.PLACE);
	}

	@Override
	protected void end() {
		interrupted();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
