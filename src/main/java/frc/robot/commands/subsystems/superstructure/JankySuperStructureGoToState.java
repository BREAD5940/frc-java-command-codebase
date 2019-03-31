// package frc.robot.commands.subsystems.superstructure;

// import com.ctre.phoenix.motorcontrol.ControlMode;

// import org.ghrobotics.lib.mathematics.units.Length;
// import org.ghrobotics.lib.mathematics.units.LengthKt;

// import org.team5940.pantry.experimental.command.SendableCommandBase;
// import edu.wpi.first.wpilibj.command.CommandGroup;
// import edu.wpi.first.wpilibj.command.ConditionalCommand;
// import edu.wpi.first.wpilibj.command.InstantCommand;
// import frc.robot.SuperStructureConstants;
// import frc.robot.lib.obj.RoundRotation2d;
// import frc.robot.states.ElevatorState;
// import frc.robot.states.IntakeAngle;
// import frc.robot.states.SuperStructureState;
// import frc.robot.subsystems.superstructure.SuperStructure;

// public class SuperstructureGoToState extends SendableCommandBaseGroup {
// 	private static final double kDefaultTimeout = 4;

// 	public SuperstructureGoToState(SuperStructureState requState) {
// 		this(requState, kDefaultTimeout);
// 	}

// 	public SuperstructureGoToState(ElevatorState eState) {
// 		this(new SuperStructureState(eState, SuperStructure.getInstance().updateState().getAngle()));
// 	}

// 	public SuperstructureGoToState(Length eState, IntakeAngle aState) {
// 		this(new SuperStructureState(new ElevatorState(eState), aState));
// 	}

// 	public SuperstructureGoToState(ElevatorState eState, IntakeAngle aState) {
// 		this(new SuperStructureState(eState, aState));
// 	}

// 	public SuperstructureGoToState(IntakeAngle aState) {
// 		this(new SuperStructureState(SuperStructure.getInstance().updateState().getElevator(), aState));
// 	}

// 	SuperStructureState mRequState;
// 	Length elevatorSetpoint;
// 	RoundRotation2d wristSetpoint, elbowSetpoint;

// 	public boolean checkPassthrough(SuperStructureState currentState, SuperStructureState mReqState) {
// 		var isPassThrough = ( currentState.getElbowAngle().getDegree() < -125 && mReqState.getElbowAngle().getDegree() > -85 ) 
// 			|| ( currentState.getElbowAngle().getDegree() > 60 && mReqState.getElbowAngle().getDegree() < -125 );

// 		return isPassThrough;
// 	}

// 	/**
// 	 * Move the superstructure to a requested state. This implements a super basic command que and stuff
// 	 * @param requState the state requested of the superstructure
// 	 * @param timeout the timeout of this command
// 	 * 
// 	 * @author Matthew Morley
// 	 * 
// 	 */
// 	public SuperstructureGoToState(SuperStructureState requState, double timeout) {
// 		addRequirements(SuperStructure.getInstance());
// 		mRequState = requState;
// 		setTimeout(timeout);

// 		var elevatorR = requState.getElevatorHeight();
// 		var proximalR = requState.getElbowAngle();
// 		var wristR = requState.getWristAngle();

// 		// check if the elbow is going to whack anything
// 		addSequential(new ConditionalCommand(new ){

// 			@Override
// 			protected boolean condition() {
// 				return checkPassthrough(SuperStructure.getInstance().getCurrentState(), requState);
// 			}
// 		});

// 		// first of all, decide if we need to move the elevator at all. We don't need to move the elevator if the elevator is already safe or if we aren't passing through
// 		addSequential(new ConditionalCommand(new WaitForElevatorSubcommand(new SuperStructureState(new ElevatorState(SuperStructureConstants.Elevator.kClearFirstStageMaxHeight)))) {
// 			@Override
// 			protected boolean condition() {
// 				var currentState = SuperStructure.getInstance().updateState();

// 				// boolean vertical;
// 				// check if the elbow is already vertical and we aren't trying to yeet into anything
// 				var vertical = ( Math.abs(requState.getElbowAngle().getDegree()-90) < 10 || Math.abs(requState.getWristAngle().getDegree()-90) < 10 );

// 				// check if we are moving up or down
// 				var deltaHeight = currentState.getElevatorHeight().minus(requState.getElevatorHeight());
// 				var upwardMove = (deltaHeight.getInch() > 0);

// 				//check if the proximal is in danger of hitting something. First, check if we are staying "deployed" (extended in the front) or not. 
// 				var deployedThreshold = 60;
// 				var isDeployed = (currentState.getElbowAngle().absoluteValueOf().getDegree() > deployedThreshold); // TODO check angles and absolute value
// 				var isStayingDeployed = isDeployed && (requState.getElbowAngle().absoluteValueOf().getDegree() > deployedThreshold);

// 				var isMoveFromDeployedToRetracted = ( currentState.getElbowAngle().getDegree() > -60 && requState.getElbowAngle().getDegree() < -30 );

// 				var isMoveFromRetractedToDeployed = (  currentState.getElbowAngle().getDegree() < -60 && requState.getElbowAngle().getDegree() > -30 );

// 				// so we need to mvoe the elevator to a safe position if we want to passthrough

// 			}
// 		}
// 		);

// 	}

// 	class StateMovementCommand extends SendableCommandBase {
// 		SuperStructureState mRequState;
// 		double kWristSetpoint, kElbowSetpoint;
// 		private boolean hasSetState = false;
// 		private final RoundRotation2d wristSetpoint, elbowSetpoint;
// 		private final Length elevatorSetpoint;

// 		public StateMovementCommand(SuperStructureState requState) {
// 			this(requState, kDefaultTimeout);
// 		}

// 		public StateMovementCommand(ElevatorState eState) {
// 			this(new SuperStructureState(eState, SuperStructure.getInstance().updateState().getAngle()));
// 		}

// 		public StateMovementCommand(Length eState, IntakeAngle aState) {
// 			this(new SuperStructureState(new ElevatorState(eState), aState));
// 		}

// 		public StateMovementCommand(ElevatorState eState, IntakeAngle aState) {
// 			this(new SuperStructureState(eState, aState));
// 		}

// 		public StateMovementCommand(IntakeAngle aState) {
// 			this(new SuperStructureState(SuperStructure.getInstance().updateState().getElevator(), aState));
// 		}

// 		/**
// 		 * Move the superstructure to a requested state. 
// 		 * @param requState the state requested of the superstructure
// 		 * @param timeout the timeout of this command
// 		 * 
// 		 * @author Matthew Morley
// 		 */
// 		public StateMovementCommand(SuperStructureState requState, double timeout) {
// 			addRequirements(SuperStructure.getInstance()); // TODO so I'm still <confuse> about reserving superstructure vs the individual parts.
// 			addRequirements(SuperStructure.getInstance().getWrist());
// 			addRequirements(SuperStructure.getInstance().getElbow());
// 			addRequirements(SuperStructure.getElevator());
// 			mRequState = requState;
// 			setTimeout(timeout);

// 			this.elevatorSetpoint = requState.getElevatorHeight();
// 			this.wristSetpoint = requState.getWristAngle();
// 			this.elbowSetpoint = requState.getElbowAngle();
// 		}

// 		// Called just before this Command runs the first time
// 		@Override
// 		public void initialize() {
// 			System.out.println("==============================================================");
// 			System.out.println("Requested move loc: ");
// 			System.out.println(String.format("Elevator (%s) elbow (%s) wrist (%s)", elevatorSetpoint.getInch(), elbowSetpoint.getDegree(), wristSetpoint.getDegree()));
// 			System.out.println("==============================================================");
// 			SuperStructure.getInstance().move(mRequState);
// 			hasSetState = true;
// 			// SuperStructure.getInstance().moveSuperstructureCombo(mRequState);
// 		}

// 		// Called repeatedly when this Command is scheduled to run
// 		@Override
// 		public void execute() {
// 			SuperStructure.getInstance().move(mRequState);
// 			hasSetState = true;
// 			// System.out.println("target state: " + mRequState.toCSV());
// 		}

// 		// Make this return true when this Command no longer needs to run execute()
// 		@Override
// 		public boolean isFinished() {
// 			if (!hasSetState)
// 				execute();
// 			return (checkElbow() && checkWrist() && checkElevator()) || isTimedOut();
// 		}

// 		private boolean checkElbow() {
// 			RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getElbow().angle;
// 			// RoundRotation2d mTarget = mRequState.getWrist().angle;
// 			double kTolerence = 5; // degrees
// 			var mError = Math.abs(elbowSetpoint.getDegree() - mCurrent.getDegree());
// 			boolean isWithin = (Math.abs(mError) < kTolerence);
// 			System.out.printf("ELBOW: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), kWristSetpoint, mError, isWithin);
// 			System.out.println("");
// 			return isWithin;
// 		}

// 		private boolean checkWrist() {
// 			RoundRotation2d mCurrent = SuperStructure.getInstance().getCurrentState().getWrist().angle;
// 			// RoundRotation2d mTarget = mRequState.getWrist().angle;
// 			double kTolerence = 5; // degrees
// 			var mError = Math.abs(wristSetpoint.getDegree() - mCurrent.getDegree());
// 			boolean isWithin = (Math.abs(mError) < kTolerence);
// 			System.out.printf("WRIST: Current %s Target %s Error %s within Tolerance %s", mCurrent.getDegree(), kWristSetpoint, mError, isWithin);
// 			System.out.println("");
// 			return isWithin;
// 		}

// 		private boolean checkElevator() {
// 			Length mCurrent = SuperStructure.getInstance().getCurrentState().getElevatorHeight();
// 			Length mTarget = mRequState.elevator.height;
// 			double kTolerence = 1.5; // inches
// 			double mError = mTarget.minus(mCurrent).getInch();
// 			boolean isWithin = (Math.abs(mError) < kTolerence);
// 			System.out.printf("ELEVATOR: Current %s Target %s Error %s within Tolerance %s", mCurrent.getInch(), mTarget.getInch(), mError, isWithin);
// 			System.out.println("");
// 			return isWithin;
// 		}

// 		// Called once after isFinished returns true
// 		@Override
// 		public void end(boolean interrupted) {
// 			System.out.println("==============================================================");
// 		}

// 		// Called when another command which requires one or more of the same
// 		// subsystems is scheduled to run
// 		@Override
// 		protected void interrupted() {}
// 	}

// 	class WaitForFinalSetpointSubcommand  extends WaitSubCommand {
// 		public WaitForFinalSetpointSubcommand (SuperStructureState endState) {
// 			super(endState);
// 			// mEndState = endState;
// 		}

// 		@Override
// 		public boolean isFinished(SuperStructureState currentState) {
// 			return Math.abs(mEndState.getElevatorHeight().getInch() - currentState.getElevatorHeight().getInch()) <= mHeightThreshold.getInch()
// 					&& Math.abs(mEndState.getElbow().angle.getDegree() - currentState.getElbow().angle.getDegree()) <= mElbowThreshold.getDegree()
// 					&& Math.abs(mEndState.getWrist().angle.getDegree() - currentState.getWrist().angle.getDegree()) <= mElbowThreshold.getDegree();
// 		}
// 	}

// 	class WaitForElbowSubcommand  extends WaitSubCommand {
// 		public WaitForElbowSubcommand (SuperStructureState endState) {
// 			super(endState);
// 			// mEndState = endState;
// 		}

// 		@Override
// 		public boolean isFinished(SuperStructureState currentState) {
// 			return Math.abs(mEndState.getElbow().angle.getDegree() - currentState.getElbow().angle.getDegree()) <= mElbowThreshold.getDegree();
// 		}
// 	}

// 	class WaitForWristSubcommand  extends WaitSubCommand {
// 		public WaitForWristSubcommand (SuperStructureState endState) {
// 			super(endState);
// 			// mEndState = endState;
// 		}

// 		@Override
// 		public boolean isFinished(SuperStructureState currentState) {
// 			return Math.abs(mEndState.getWrist().angle.getDegree() - currentState.getWrist().angle.getDegree()) <= mElbowThreshold.getDegree();
// 		}
// 	}

// 	class WaitForElevatorSubcommand extends WaitSubCommand {
// 		public WaitForElevatorSubcommand (SuperStructureState endState) {
// 			super(endState);
// 			// mEndState = endState;
// 		}

// 		@Override
// 		public boolean isFinished(SuperStructureState currentState) {
//   		return Math.abs(mEndState.getElevatorHeight().getInch() - currentState.getElevatorHeight().getInch()) <= mHeightThreshold.getInch();
// 		}
// 	}

// 	class SetWristSetpoint extends InstantCommand {
// 		RoundRotation2d mSetpoint;
// 		public SetWristSetpoint(RoundRotation2d setpoint) {
// 			this.mSetpoint = setpoint;
// 		}

// 		@Override
// 		public void initialize() {
// 			SuperStructure.getInstance().getWrist().requestAngle(ControlMode.MotionMagic, mSetpoint);
// 		}
// 	}

// 	class SetElbowSetpoint extends InstantCommand {
// 		RoundRotation2d mSetpoint;
// 		public SetElbowSetpoint(RoundRotation2d setpoint) {
// 			this.mSetpoint = setpoint;
// 		}

// 		@Override
// 		public void initialize() {
// 			SuperStructure.getInstance().getElbow().requestAngle(ControlMode.MotionMagic, mSetpoint);
// 		}
// 	}

// 	class SetElevatorSetpoint extends InstantCommand {
// 		SuperStructureState mSetpoint;
// 		public SetElevatorSetpoint(SuperStructureState setpoint) {
// 			this.mSetpoint = setpoint;
// 		}

// 		@Override
// 		public void initialize() {
// 			SuperStructure.getElevator().setPositionSetpoint(mSetpoint);
// 		}
// 	}

// 	class SetIntakeAnglesSetpoint extends InstantCommand {
// 		SuperStructureState mSetpoint;
// 		public SetIntakeAnglesSetpoint(SuperStructureState setpoint) {
// 			this.mSetpoint = setpoint;
// 		}

// 		@Override
// 		public void initialize() {
// 			SuperStructure.getInstance().getElbow().requestAngle(ControlMode.MotionMagic, mSetpoint);
// 			SuperStructure.getInstance().getWrist().requestAngle(ControlMode.MotionMagic, mSetpoint);

// 		}
// 	}

// 	abstract class WaitSubCommand extends SendableCommandBase {
// 		SuperStructureState mEndState, currentState;
// 		public Length mHeightThreshold = LengthKt.getInch(1.0);
// 		public RoundRotation2d mElbowThreshold = RoundRotation2d.getDegree(5.0);
// 		public RoundRotation2d mWristThreshold = RoundRotation2d.getDegree(5.0);

// 		public WaitSubCommand(SuperStructureState requ_) {
// 			this.mEndState = requ_;
// 		}

// 		public void feedState(SuperStructureState currentState){ 
// 			this.currentState = currentState;
// 		}

// 		@Override
// 		public boolean isFinished() {
// 			feedState(SuperStructure.getInstance().getCurrentState());
// 			return isFinished(currentState);
// 		}

// 		abstract boolean isFinished(SuperStructureState mCurrent);

// 		@Override
// 		public void initialize() {}

// 		@Override
// 		public void execute() {}

// 		@Override
// 		public void end(boolean interrupted) {}

// 		@Override
// 		protected void interrupted() {}
// 	}

// }
