package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.Logger;
import frc.robot.lib.obj.RoundRotation2d;
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
		System.out.println("requested elbow angle: " + mRequState.getElbowAngle().getDegree());
		System.out.println("current elbow angle: " + SuperStructure.getInstance().getCurrentState().getElbowAngle().getDegree());
		System.out.println("==============================================================");
		SuperStructure.getInstance().moveSuperstructureCombo(mRequState);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		SuperStructure.getInstance().move(mRequState);
		// System.out.println("target state: " + mRequState.toCSV());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (checkElbow() && checkWrist() && checkElevator()) || isTimedOut();
	}

	private boolean checkElbow() {
		RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getElbowAngle();
		RoundRotation2d mTarget = mRequState.getElbowAngle();
		double kTolerence = 5; // degrees
		double mError = mTarget.minus(mCurrent).getDegree();
		boolean isWithin = (Math.abs(mError) < kTolerence);
		System.out.printf("ELBOW: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), mTarget.getDegree(), mError, isWithin);
		System.out.println(" ");
		return isWithin;
	}

	private boolean checkWrist() {
		RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getWrist().angle;
		RoundRotation2d mTarget = mRequState.getWrist().angle;
		double kTolerence = 5; // degrees
		double mError = mTarget.minus(mCurrent).getDegree();
		boolean isWithin = (Math.abs(mError) < kTolerence);
		System.out.printf("WRIST: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), mTarget.getDegree(), mError, isWithin);
		System.out.println("------------------------------");
		return isWithin;
	}

	private boolean checkElevator() {
		Length mCurrent = SuperStructure.getInstance().getCurrentState().getElevatorHeight();
		Length mTarget = mRequState.elevator.height;
		double kTolerence = 2; // inches
		double mError = mTarget.minus(mCurrent).getInch();
		boolean isWithin = (Math.abs(mError) < kTolerence);
		System.out.printf("ELEVATOR: Current %s Target %s Error %s within Tolerance %s", mCurrent.getInch(), mTarget.getInch(), mError, isWithin);
		System.out.println("------------------------------");
		return isWithin;
	}


	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
