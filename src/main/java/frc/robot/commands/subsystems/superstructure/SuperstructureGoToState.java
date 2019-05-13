package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;

import org.team5940.pantry.exparimental.command.SendableCommandBase;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;
import org.team5940.pantry.exparimental.command.WaitCommand;

public class SuperstructureGoToState extends WaitCommand {
	SuperStructureState mRequState;
	double kWristSetpoint, kElbowSetpoint;
	private static final double kDefaultTimeout = 4;
	private boolean hasSetState = false;
	private final RoundRotation2d wristSetpoint, elbowSetpoint;
	private final Length elevatorSetpoint;

	public SuperstructureGoToState(SuperStructureState requState) {
		this(requState, kDefaultTimeout);
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
		super(timeout);

		addRequirements(SuperStructure.getInstance());
		addRequirements(SuperStructure.getInstance().getWrist());
		addRequirements(SuperStructure.getInstance().getElbow());
		addRequirements(SuperStructure.getElevator());
		mRequState = requState;


		this.elevatorSetpoint = requState.getElevatorHeight();
		this.wristSetpoint = requState.getWristAngle();
		this.elbowSetpoint = requState.getElbowAngle();
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		SuperStructure.getInstance().move(mRequState);
		hasSetState = true;

		super.initialize();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		SuperStructure.getInstance().move(mRequState);
		hasSetState = true;

		super.execute();
	}

	@Override
	public boolean isFinished() {
		if (!hasSetState)
			execute();
		return (checkElbow() && checkWrist() && checkElevator()) || super.isFinished();
	}

	private boolean checkElbow() {
		RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getElbow().angle;
		// RoundRotation2d mTarget = mRequState.getWrist().angle;
		double kTolerence = 5; // degrees
		var mError = Math.abs(elbowSetpoint.getDegree() - mCurrent.getDegree());
		boolean isWithin = (Math.abs(mError) < kTolerence);
		//System.out.printf("ELBOW: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), kWristSetpoint, mError, isWithin);
		//System.out.println("");
		return isWithin;
	}

	private boolean checkWrist() {
		RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getWrist().angle;
		// RoundRotation2d mTarget = mRequState.getWrist().angle;
		double kTolerence = 5; // degrees
		var mError = Math.abs(wristSetpoint.getDegree() - mCurrent.getDegree());
		boolean isWithin = (Math.abs(mError) < kTolerence);
		//System.out.printf("WRIST: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), kWristSetpoint, mError, isWithin);
		//System.out.println("");
		return isWithin;
	}

	private boolean checkElevator() {
		Length mCurrent = SuperStructure.getInstance().getCurrentState().getElevatorHeight();
		Length mTarget = mRequState.elevator.height;
		double kTolerence = 1; // inches
		double mError = mTarget.minus(mCurrent).getInch();
		boolean isWithin = (Math.abs(mError) < kTolerence);
		//System.out.printf("ELEVATOR: Current %s Target %s Error %s within Tolerance %s", mCurrent.getInch(), mTarget.getInch(), mError, isWithin);
		//System.out.println("");
		return isWithin;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		var currentState = SuperStructure.getInstance().getCurrentState();
		SuperStructure.getInstance().move(currentState);

		super.end(interrupted);
	}

}
