package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.Logger;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
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

	public SuperstructureGoToState(ElevatorState eState) {
		this(new SuperStructureState(eState, SuperStructure.getInstance().updateState().getAngle()));
	}

	public SuperstructureGoToState(IntakeAngle aState) {
		this(new SuperStructureState(SuperStructure.getInstance().updateState().getElevator(), aState));
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

		//former superstructure periodic
		SuperStructure.getInstance().updateState();

		SuperStructureState prevState = SuperStructure.getInstance().lastState;
		// double mCurrentWristTorque = Math.abs(SuperStructure.getInstance().calculateWristTorque(prevState)); // torque due to gravity and elevator acceleration, newton meters
		// double mCurrentElbowTorque = Math.abs(SuperStructure.getInstance().calculateElbowTorques(prevState, mCurrentWristTorque)); // torque due to gravity and elevator acceleration, newton meters

		// double wristVoltageGravity = SuperStructure.getInstance().getWTransmission().getVoltageForTorque(SuperStructure.getInstance().updateState().getWrist().velocity.getValue(), mCurrentWristTorque);
		// double elbowVoltageGravity = SuperStructure.getInstance().getETransmission().getVoltageForTorque(SuperStructure.getInstance().updateState().getElbow().velocity.getValue(), mCurrentElbowTorque);
		double elevatorPercentVbusGravity = SuperStructure.getInstance().getElevator().getVoltage(SuperStructure.getInstance().updateState()) / 12;//getElevator().getMaster().getBusVoltage();		

		// if (Math.abs(mOI.getWristAxis()) > 0.07) {
		// SuperStructure.getInstance().getWrist().getMaster().set(ControlMode.Position, mRequState.getWrist().angle);

		// SuperStructure.getInstance().getElbow().getMaster().set(ControlMode.Position, mRequState.getElbow().angle);
		SuperStructureState stateSetpoint = SuperStructure.getInstance().plan(mRequState);

		SuperStructure.getInstance().getWrist().requestAngle(stateSetpoint.getWrist().angle); // div by 12 because it expects a throttle
		SuperStructure.getInstance().getElbow().requestAngle(stateSetpoint.getElbow().angle); // div by 12 because it expects a throttle
		SuperStructure.getInstance().getElevator().setPositionArbitraryFeedForward(stateSetpoint.getElevator().height, elevatorPercentVbusGravity / 12d);
		// getElevator().getMaster().set(ControlMode.PercentOutput, elevatorPercentVbusGravity);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		Logger.log("elevator within tolerence? " + ((Math.abs(mRequState.getElevatorHeight().getInch() - SuperStructure.getInstance().updateState().getElevatorHeight().getInch()) <= 0.5)));
		Logger.log("elbow within tolerence? " + (Math.abs(mRequState.getElbow().angle.getDegree() - SuperStructure.getInstance().updateState().getElbow().angle.getDegree()) <= 5));
		Logger.log("wrist within tolerence? " + (Math.abs(mRequState.getWrist().angle.getDegree() - SuperStructure.getInstance().updateState().getWrist().angle.getDegree()) <= 5));
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
