//package frc.robot.lib.statemachines;
//
//import org.team5940.pantry.exparimental.command.SendableCommandBase;
//import frc.robot.Robot;
//import frc.robot.lib.statemachines.AutoMotionStateMachine.MotionType;
//
///**
// * is literally a toggle
// *
// * @author jocleyn McHugo
// */
//public class PickupToggle extends SendableCommandBase {
//	public PickupToggle() {}
//
//	@Override
//	public void initialize() {
//		Robot.autoState.setMotionType(MotionType.PICKUP);
//	}
//
//	@Override
//	protected void interrupted() {
//		Robot.autoState.setMotionType(MotionType.PLACE);
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
