package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperstructureGoToState extends CommandGroup {
	private static final double kDefaultTimeout = 4;

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

	SuperStructureState mRequState;
	Length elevatorSetpoint;
	RoundRotation2d wristSetpoint, elbowSetpoint;

	/**
	 * Move the superstructure to a requested state.
	 * @param requState the state requested of the superstructure
	 * @param timeout the timeout of this command
	 * 
	 * @author Matthew Morley
	 */
	public SuperstructureGoToState(SuperStructureState requState, double timeout) {

		mRequState = requState;
		setTimeout(timeout);

		this.elevatorSetpoint = requState.getElevatorHeight();
		this.wristSetpoint = requState.getWristAngle();
		this.elbowSetpoint = requState.getElbowAngle();
	}

	class StateMovementCommand extends Command {
		SuperStructureState mRequState;
		double kWristSetpoint, kElbowSetpoint;
		private boolean hasSetState = false;
		private final RoundRotation2d wristSetpoint, elbowSetpoint;
		private final Length elevatorSetpoint;

		public StateMovementCommand(SuperStructureState requState) {
			this(requState, kDefaultTimeout);
		}

		public StateMovementCommand(ElevatorState eState) {
			this(new SuperStructureState(eState, SuperStructure.getInstance().updateState().getAngle()));
		}

		public StateMovementCommand(Length eState, IntakeAngle aState) {
			this(new SuperStructureState(new ElevatorState(eState), aState));
		}

		public StateMovementCommand(ElevatorState eState, IntakeAngle aState) {
			this(new SuperStructureState(eState, aState));
		}

		public StateMovementCommand(IntakeAngle aState) {
			this(new SuperStructureState(SuperStructure.getInstance().updateState().getElevator(), aState));
		}

		/**
		 * Move the superstructure to a requested state. 
		 * @param requState the state requested of the superstructure
		 * @param timeout the timeout of this command
		 * 
		 * @author Matthew Morley
		 */
		public StateMovementCommand(SuperStructureState requState, double timeout) {
			requires(SuperStructure.getInstance()); // TODO so I'm still <confuse> about reserving superstructure vs the individual parts.
			requires(SuperStructure.getInstance().getWrist());
			requires(SuperStructure.getInstance().getElbow());
			requires(SuperStructure.getElevator());
			mRequState = requState;
			setTimeout(timeout);

			this.elevatorSetpoint = requState.getElevatorHeight();
			this.wristSetpoint = requState.getWristAngle();
			this.elbowSetpoint = requState.getElbowAngle();
		}

		// Called just before this Command runs the first time
		@Override
		protected void initialize() {
			System.out.println("==============================================================");
			System.out.println("Requested move loc: ");
			System.out.println(String.format("Elevator (%s) elbow (%s) wrist (%s)", elevatorSetpoint.getInch(), elbowSetpoint.getDegree(), wristSetpoint.getDegree()));
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
			if (!hasSetState)
				execute();
			return (checkElbow() && checkWrist() && checkElevator()) || isTimedOut();
		}

		private boolean checkElbow() {
			RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getElbow().angle;
			// RoundRotation2d mTarget = mRequState.getWrist().angle;
			double kTolerence = 5; // degrees
			var mError = Math.abs(elbowSetpoint.getDegree() - mCurrent.getDegree());
			boolean isWithin = (Math.abs(mError) < kTolerence);
			System.out.printf("ELBOW: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), kWristSetpoint, mError, isWithin);
			System.out.println("");
			return isWithin;
		}

		private boolean checkWrist() {
			RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getWrist().angle;
			// RoundRotation2d mTarget = mRequState.getWrist().angle;
			double kTolerence = 5; // degrees
			var mError = Math.abs(wristSetpoint.getDegree() - mCurrent.getDegree());
			boolean isWithin = (Math.abs(mError) < kTolerence);
			System.out.printf("WRIST: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), kWristSetpoint, mError, isWithin);
			System.out.println("");
			return isWithin;
		}

		private boolean checkElevator() {
			Length mCurrent = SuperStructure.getInstance().getCurrentState().getElevatorHeight();
			Length mTarget = mRequState.elevator.height;
			double kTolerence = 1.5; // inches
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

	class FullWaitSubCommand extends WaitSubCommand {
		public FullWaitSubCommand(SuperStructureState endState) {
			super(endState);
			// mEndState = endState;
		}

		public SuperStructureState mEndState;
		public Length mHeightThreshold = LengthKt.getInch(1.0);
		public RoundRotation2d mElbowThreshold = RoundRotation2d.getDegree(5.0);

		@Override
		public boolean isFinished(SuperStructureState currentState) {
			return Math.abs(mEndState.getElevatorHeight().getInch() - currentState.getElevatorHeight().getInch()) <= mHeightThreshold.getInch()
					&& Math.abs(mEndState.getElbow().angle.getDegree() - currentState.getElbow().angle.getDegree()) <= mElbowThreshold.getDegree()
					&& Math.abs(mEndState.getWrist().angle.getDegree() - currentState.getWrist().angle.getDegree()) <= mElbowThreshold.getDegree();
		}
	}

	abstract class WaitSubCommand extends Command {
		SuperStructureState mReqState, currentState;

		public WaitSubCommand(SuperStructureState requ_) {
			this.mReqState = requ_;
		}

		public void feedState(SuperStructureState new_){ 
			this.currentState = new_;
		}

		@Override
		protected boolean isFinished() {
			return isFinished(currentState);
		}

		abstract boolean isFinished(SuperStructureState mCurrent);

		@Override
		protected void initialize() {}
	
		@Override
		protected void execute() {}

		@Override
		protected void end() {}
	
		@Override
		protected void interrupted() {}
	}

	


}
