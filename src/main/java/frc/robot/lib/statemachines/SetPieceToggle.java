package frc.robot.lib.statemachines;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.Robot;
import frc.robot.lib.statemachines.AutoMotionStateMachine.HeldPiece;

/**
 * is literally a toggle
 * 
 * @author jocleyn McHugo
 */
public class SetPieceToggle extends SendableCommandBase {
	public SetPieceToggle() {}

	@Override
	public void initialize() {
		Robot.autoState.setHeldPiece(HeldPiece.CARGO);
	}

	public void interrupted() {
		Robot.autoState.setHeldPiece(HeldPiece.HATCH);
	}

	@Override
	public void end(boolean interrupted) {
		interrupted();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
