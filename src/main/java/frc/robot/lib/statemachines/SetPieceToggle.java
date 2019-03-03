package frc.robot.lib.statemachines;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.statemachines.AutoMotionStateMachine.HeldPiece;

/**
 * is literally a toggle
 * 
 * @author jocleyn McHugo
 */
public class SetPieceToggle extends Command {
	public SetPieceToggle() {}

	@Override
	protected void initialize() {
		Robot.autoState.setHeldPiece(HeldPiece.CARGO);
	}

	@Override
	protected void interrupted() {
		Robot.autoState.setHeldPiece(HeldPiece.HATCH);
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
