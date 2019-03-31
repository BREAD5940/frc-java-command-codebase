package frc.robot.lib.statemachines;

import org.team5940.pantry.experimental.command.SendableCommandBase;

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

	@Override
	public void end(boolean interrupted) {
		Robot.autoState.setHeldPiece(HeldPiece.HATCH);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
