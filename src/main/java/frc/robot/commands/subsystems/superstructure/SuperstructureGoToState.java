package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperstructureGoToState extends Command {
	SuperStructureState mRequState;
	double kWristSetpoint, kElbowSetpoint;
	private double kDefaultTimeout = 5;
	private boolean hasSetState = false;

	public SuperstructureGoToState(SuperStructureState requState) {
		requires(SuperStructure.getInstance());
		mRequState = requState;
		setTimeout(kDefaultTimeout);
	}

	public SuperstructureGoToState(ElevatorState eState) {
		this(new SuperStructureState(eState, SuperStructure.getInstance().updateState().getAngle()));
	}

	public SuperstructureGoToState(Length eState, IntakeAngle aState) {
		this(new SuperStructureState(new ElevatorState(eState), aState));
	}

	public SuperstructureGoToState(ElevatorState eState, IntakeAngle aState) {
		this(new SuperStructureState(eState, aState));
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
		System.out.println("==============================================================");
		System.out.println("Requested move loc: " + mRequState.getCSVHeader());
		System.out.println(mRequState.toCSV());
		System.out.println("==============================================================");
		SuperStructure.getInstance().move(mRequState);
		hasSetState = true;
		// SuperStructure.getInstance().moveSuperstructureCombo(mRequState);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		SuperStructure.getInstance().move(mRequState);
		hasSetState = true;
		// System.out.println("target state: " + mRequState.toCSV());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if(!hasSetState) execute();
		return (checkElbow() && checkWrist() && checkElevator()) || isTimedOut();
	}

	private boolean checkElbow() {
		RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getElbow().angle;
		// RoundRotation2d mTarget = mRequState.getWrist().angle;
		double kTolerence = 5; // degrees
		var mError = SuperStructure.getInstance().getElbow().getMaster().getRotation2dError();
		boolean isWithin = (Math.abs(mError.getDegree()) < kTolerence);
		System.out.printf("ELBOW: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), kWristSetpoint, mError.getDegree(), isWithin);
		System.out.println("");
		return isWithin;
	}

	private boolean checkWrist() {
		RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getWrist().angle;
		// RoundRotation2d mTarget = mRequState.getWrist().angle;
		double kTolerence = 5; // degrees
		var mError = SuperStructure.getInstance().getElbow().getMaster().getRotation2dError();
		boolean isWithin = (Math.abs(mError.getDegree()) < kTolerence);
		System.out.printf("WRIST: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), kWristSetpoint, mError.getDegree(), isWithin);
		System.out.println("");
		return isWithin;
	}

	private boolean checkElevator() {
		Length mCurrent = SuperStructure.getInstance().getCurrentState().getElevatorHeight();
		Length mTarget = mRequState.elevator.height;
		double kTolerence = 1; // inches
		double mError = mTarget.minus(mCurrent).getInch();
		boolean isWithin = (Math.abs(mError) < kTolerence);
		System.out.printf("ELEVATOR: Current %s Target %s Error %s within Tolerance %s", mCurrent.getInch(), mTarget.getInch(), mError, isWithin);
		System.out.println("");
		return isWithin;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("==============================================================");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
