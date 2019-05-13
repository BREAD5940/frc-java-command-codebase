// hi it's matt again
// this file's unused, so i commented it out
// so i dont have to fix it
// also, it's not technically a state machine I think, because it doesn't hold any onformation
// about a current state? either way, its easy to fix but im lazy

//package frc.robot.lib.statemachines;
//
//import org.team5940.pantry.exparimental.command.SendableCommandBase;
//import frc.robot.Robot;
//import frc.robot.lib.statemachines.AutoMotionStateMachine.HeldPiece;
//
///**
// * is literally a toggle
// *
// * @author jocleyn McHugo
// */
//public class SetPieceToggle extends SendableCommandBase {
//	public SetPieceToggle() {}
//
//	@Override
//	public void initialize() {
//		Robot.autoState.setHeldPiece(HeldPiece.CARGO);
//	}
//
//	@Override
//	protected void interrupted() {
//		Robot.autoState.setHeldPiece(HeldPiece.HATCH);
//	}
//
//	@Override
//	protected void end() {
//		interrupted();
//	}
//
//	@Override
//	public boolean isFinished() {
//		return false;
//	}
//}
