package frc.robot.commands.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperstructureGoToState extends Command {
	SuperStructureState mRequState;
	private double kDefaultTimeout = 2;

	public SuperstructureGoToState(SuperStructureState requState) {
		requires(SuperStructure.getInstance());
		mRequState = requState;
		setTimeout(kDefaultTimeout);
	}

	public SuperstructureGoToState(SuperStructureState requState, double timeout) {
		requires(SuperStructure.getInstance());
		mRequState = requState;
		setTimeout(timeout);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		SuperStructure.getInstance().moveSuperstructureCombo(mRequState);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// if (Math.abs(mOI.getWristAxis()) > 0.07) {
			SuperStructure.getInstance().getWrist().getMaster().set(ControlMode.Position, mRequState.getWrist().angle);

			SuperStructure.getInstance().getElbow().getMaster().set(ControlMode.Position, mRequState.getElbow().angle);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return ((Math.abs(mRequState.getElevatorHeight().getInch() - SuperStructure.getInstance().updateState().getElevatorHeight().getInch()) <= 0.5)
				&& (Math.abs(mRequState.getElbow().angle.getDegree() - SuperStructure.getInstance().updateState().getElbow().angle.getDegree()) <= 5) //FIXME check tolerences
				&& (Math.abs(mRequState.getWrist().angle.getDegree() - SuperStructure.getInstance().updateState().getWrist().angle.getDegree()) <= 5)) || isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
