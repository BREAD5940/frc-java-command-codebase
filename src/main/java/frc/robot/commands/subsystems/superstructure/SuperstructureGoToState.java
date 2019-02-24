package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.Logger;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperstructureGoToState extends Command {
	SuperStructureState mRequState;
	private double kDefaultTimeout = 5;

	public SuperstructureGoToState(SuperStructureState requState) {
		requires(SuperStructure.getInstance());
		mRequState = requState;
		setTimeout(kDefaultTimeout);
	}

	public SuperstructureGoToState(ElevatorState eState) {
		this(new SuperStructureState(eState, SuperStructure.getInstance().updateState().getAngle()));
	}

	public SuperstructureGoToState(IntakeAngle aState) {
		this(new SuperStructureState(SuperStructure.getInstance().updateState().getElevator(), aState));
	}

	public SuperstructureGoToState(ElevatorState eState, IntakeAngle aState) {
		this(new SuperStructureState(eState, aState));
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
		SuperStructure.getInstance().move(mRequState);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		Logger.log("elevator: " + SuperStructure.getInstance().updateState().getElevatorHeight().getInch() + "target: " + (mRequState.getElevatorHeight().getInch()));
		Logger.log("elevator within tolerence? " + ((Math.abs(mRequState.getElevatorHeight().getInch() - SuperStructure.getInstance().updateState().getElevatorHeight().getInch()) <= 0.5)));
		Logger.log("elbow within tolerence? " + (Math.abs(mRequState.getElbow().angle.getDegree() - SuperStructure.getInstance().updateState().getElbow().angle.getDegree()) <= 5));
		Logger.log("wrist within tolerence? " + (Math.abs(mRequState.getWrist().angle.getDegree() - SuperStructure.getInstance().updateState().getWrist().angle.getDegree()) <= 5));
		return ((Math.abs(mRequState.getElevatorHeight().getInch() - SuperStructure.getInstance().updateState().getElevatorHeight().getInch()) <= 0.5 * 5)
				&& (Math.abs(mRequState.getElbow().angle.getDegree() - SuperStructure.getInstance().updateState().getElbow().angle.getDegree()) <= 5 * 3) //FIXME check tolerences
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
