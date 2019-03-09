
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.SuperStructureConstants;
import frc.robot.lib.Logger;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

/**
 * Instructions for using this mutant command-thing:
 *  - Do NOT call the constructor (unless ur OI)
 *  - Call the planner
 *  - Call testableSSMotion.getInstance().start();
 *    - This will iterate through the planned commandQueue
 *    - It'll end when it's done
 * 
 * @author Jocelyn McHugo
 */
public class testableSSMotion /*extends Command*/ {
	/* RELEVANT COMMANDS:
	  - ElevatorMove
	  - ArmMove
	  - ArmWaitForElevator
	 */

	boolean isReal = false;
	private SuperStructureState gsIn, currentState;

	private void requires(Object o) {}

	@Deprecated
	private testableSSMotion() {
		// requires(SuperStructure.getInstance());

		// requires(SuperStructure.getInstance().getWrist());
		// requires(SuperStructure.getInstance().getElbow());
		// requires(SuperStructure.getElevator());
	}

	public testableSSMotion(SuperStructureState gsIn, SuperStructureState current) {
		System.out.println("ssmotion instan");
		this.gsIn = gsIn;
		this.currentState = current;
		// requires(SuperStructure.getInstance().getWrist());
		// requires(SuperStructure.getInstance().getElbow());
		// requires(SuperStructure.getElevator());
		// requires(SuperStructure.getInstance());

		plan(this.gsIn, currentState);
		Logger.log(String.format("Start state (%s) Goal state (%s)", currentState.toString(), gsIn.toString()));
		System.out.println("===================================================================");
		Logger.log(String.format("Start state (%s)", currentState.toString()));

		// Logger.log(getQueue().getCommandLog().get(0));
		for (String s : getQueue().getCommandLog()) {
			System.out.println(s);
		}
		Logger.log(String.format("End state (%s)", gsIn.toString()));

		System.out.println("===================================================================");
	}

	public testableSSMotion(SuperStructureState gsIn) {
		this(gsIn, SuperStructure.getInstance().updateState());
	}

	private static testableSSMotion instance_;
	protected TestableCommandGroup queue = new TestableCommandGroup();
	// protected CommandGroup eleQueue = new CommandGroup();
	// protected CommandGroup armQueue = new CommandGroup();
	protected Optional<Command> current;

	@Deprecated
	public static testableSSMotion getInstance() {
		if (instance_ == null) {
			instance_ = new testableSSMotion();
		}
		return instance_;
	}

	/**
	 * Get the absolute angle of the wrist given a random angle per encoder and the proximal angle
	 * @param dumbWrist the wrist angle as the encoder/presets sees it
	 * @param relevantProx the proximal angle
	 * @return the absolute angle of the wrist
	 */
	private static RoundRotation2d getUnDumbWrist(RoundRotation2d dumbWrist, RoundRotation2d relevantProx) {
		var compensatedAngle = dumbWrist.plus(relevantProx.div(2));
		return compensatedAngle;
	}

	public boolean plan(SuperStructureState gsIn, SuperStructureState currentState) {
		System.out.println("In planner");
		var goalState = new SuperStructureState(gsIn);
		Length startArmTol = LengthKt.getInch(3);
		//CHECK if the current and goal match
		if (goalState.isEqualTo(currentState)) {
			Logger.log("Goal and current states same.");
			return true;
		}

		Logger.log("Checking basic mins/maxes");
		//SAFE illegal inputs
		if (goalState.getElevatorHeight().getInch() > SuperStructureConstants.Elevator.top.getInch() - SuperStructureConstants.Elevator.carriageHeight.getInch()) {
			Logger.log("Elevator too high! Safing elevator...");
			goalState.getElevator().setHeight(SuperStructureConstants.Elevator.top); // constrain elevator to max height
		} else if (goalState.getElevatorHeight().getInch() < SuperStructureConstants.Elevator.bottom.getInch()) {
			Logger.log("Elevator too low! Safing elevator");
			goalState.getElevator().setHeight(SuperStructureConstants.Elevator.bottom); // constrain elevator to min height
		}

		if (goalState.getElbowAngle().getDegree() > SuperStructureConstants.Elbow.kElbowMax.getDegree()) {
			Logger.log("Elbow big");
			goalState.getElbow().setAngle(SuperStructureConstants.Elbow.kElbowMax); // Constrain elbow to max
		} else if (goalState.getElbowAngle().getDegree() < SuperStructureConstants.Elbow.kElbowMin.getDegree()) {
			Logger.log("Elbow small");
			goalState.getElbow().setAngle(SuperStructureConstants.Elbow.kElbowMin); // Constrain elbow to min
		}

		Logger.log("Out of illegal safing");

		// TODO safe the wrist, which is stupid and changes a lot. Maybe we need a equation or something for it?

		//DEFINE the three goal points -- elevator, wrist, and end of intake
		Translation2d GPelevator = new Translation2d(LengthKt.getInch(0), goalState.getElevatorHeight()); // TODO maybe change constructor to use a Translation2d fromed from a Length and Rotation2d?
		Translation2d GPwrist = new Translation2d(SuperStructureConstants.Elbow.carriageToIntake, goalState.getElbowAngle().toRotation2d()).plus(GPelevator);

		Logger.log("Goal position: " + goalState.toCSV() + String.format("gpWrist: x %s y %s", GPwrist.getX().getInch(), GPwrist.getY().getInch()));

		/** 
		 * goal point end of intake 
		 */
		Translation2d GPeoi = new Translation2d(LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()),
				LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getSin() * SuperStructureConstants.Wrist.intakeOut.getInch())).plus(GPwrist);
		// Translation2d GPeoi = new Translation2d(LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()),
		// LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getSin() * SuperStructureConstants.Wrist.intakeOut.getInch())).plus(GPwrist);

		//DEFINE the three start points -- elevator, wrist, and end of intake
		Translation2d SPelevator = new Translation2d(LengthKt.getInch(0), currentState.getElevatorHeight());
		// Translation2d SPwrist = new Translation2d(LengthKt.getInch(currentState.getElbowAngle().getCos() * SuperStructureConstants.Elbow.carriageToIntake.getInch()),
		// LengthKt.getInch(currentState.getElbowAngle().getSin() * SuperStructureConstants.Elbow.carriageToIntake.getInch())).plus(SPelevator);

		Translation2d SPwrist = new Translation2d(SuperStructureConstants.Elbow.carriageToIntake, currentState.getElbowAngle().toRotation2d()).plus(SPelevator);

		Translation2d SPeoi = new Translation2d(LengthKt.getInch(getUnDumbWrist(currentState.getWristAngle(), currentState.getElbowAngle()).getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()),
				LengthKt.getInch(getUnDumbWrist(currentState.getWristAngle(), currentState.getElbowAngle()).getSin() * SuperStructureConstants.Wrist.intakeOut.getInch())).plus(SPwrist);

		Logger.log("made points");

		if (Math.atan((GPeoi.getY().getInch() + GPwrist.getY().getInch()) / (GPeoi.getX().getInch() + GPwrist.getX().getInch())) > SuperStructureConstants.Wrist.kWristMax.getRadian()) {
			Logger.log("Wrist big");
			goalState.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMax); // constrain wrist to max
		} else if (Math.atan((GPeoi.getY().getInch() + GPwrist.getY().getInch()) / (GPeoi.getX().getInch() + GPwrist.getX().getInch())) < SuperStructureConstants.Wrist.kWristMin.getRadian()) {
			Logger.log("Wrist small");
			goalState.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMin); // constrain wrist to min
		}

		//SAFE potential crashes on the end state (FIXME not necessarily in between states?)
		if (GPwrist.getY().getInch() < SuperStructureConstants.Elevator.electronicsHeight.getInch() || GPeoi.getY().getInch() < SuperStructureConstants.Elevator.electronicsHeight.getInch()) {
			Logger.log("intake too low");
			RoundRotation2d tempTheta = goalState.getElbowAngle();
			tempTheta = RoundRotation2d.getRadian(
					Math.asin(
							Math.abs(SuperStructureConstants.Elevator.electronicsHeight.getInch() - GPelevator.getY().getInch())
									/ SuperStructureConstants.Elbow.carriageToIntake.getInch()));
			goalState.getElbow().setAngle(tempTheta);
			GPwrist = new Translation2d(LengthKt.getInch(tempTheta.getCos() * SuperStructureConstants.Elbow.carriageToIntake.getInch()),
					LengthKt.getInch(Math.sin(tempTheta.getRadian()) * SuperStructureConstants.Elbow.carriageToIntake.getInch()).plus(GPelevator.getY()));
			GPeoi = new Translation2d(LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getX()),
					LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getSin() * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getY()));
		}

		//I really hope we don't actually need this
		// if (GPeoi.getY().minus(Wrist.intakeAbove).getInch() < SuperStructureConstants.Elevator.electronicsHeight.getInch()) {
		// 	Logger.log("intake still too low");
		// 	RoundRotation2d tempTheta = getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle());
		// 	tempTheta = RoundRotation2d.getRadian(
		// 			Math.asin(
		// 					(GPeoi.getY().plus(Wrist.intakeAbove).getInch() - GPwrist.getY().getInch())
		// 							/ SuperStructureConstants.Wrist.intakeOut.getInch()));
		// 	goalState.getWrist().setAngle(tempTheta);
		// 	GPeoi = new Translation2d(LengthKt.getInch(tempTheta.getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getX()),
		// 			LengthKt.getInch(Math.sin(tempTheta.getRadian()) * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getY()));
		// }

		Logger.log("a r c s i n");

		//FIND the lowest goal and end points
		Translation2d lowestGP = GPeoi;
		for (Translation2d current : Arrays.asList(GPelevator, GPwrist, GPeoi,
				GPeoi.minus(new Translation2d(LengthKt.getInch(0), SuperStructureConstants.Wrist.intakeAbove)))) {
			lowestGP = (lowestGP.getY().getInch() >= current.getY().getInch()) ? current : lowestGP;
		}

		Translation2d lowestSP = SPeoi;
		for (Translation2d current : Arrays.asList(SPelevator, SPwrist, SPeoi,
				SPeoi.minus(new Translation2d(LengthKt.getInch(0), SuperStructureConstants.Wrist.intakeAbove)))) {
			lowestSP = (lowestSP.getY().getInch() >= current.getY().getInch()) ? current : lowestSP;
		}

		Logger.log("lowests");

		//SAFE potential crashes IN BETWEEN states
		if (lowestGP.getY().getInch() < GPelevator.getY().getInch()) {
			//FIND how much of the intake is in the way

			//SET the tolerance to that number

		}

		startArmTol = GPelevator.getY().minus(LengthKt.getInch(Math.abs(lowestSP.getY().minus(SPelevator.getY()).getInch())));

		startArmTol = (startArmTol.getInch() > (Math.abs(GPelevator.getY().getInch() - Math.abs(lowestGP.getY().minus(GPelevator.getY()).getInch()))))
				? startArmTol
				: (LengthKt.getInch(Math.abs(GPelevator.getY().getInch() - Math.abs(lowestGP.getY().minus(GPelevator.getY()).getInch()))));
		Logger.log("tolerances");
		//CLEAR the queue
		this.queue = new TestableCommandGroup();
		Logger.log("queue cleared");

		//CHECK if the elevator point is in proximity to the crossbar - if it is, stow it
		// This is the VERY FIRST thing we do so that we make sure that we don't slap a meme
		if ((GPelevator.getY().getInch() < SuperStructureConstants.Elevator.crossbarBottom.getInch()
				&& SPelevator.getY().getInch() > SuperStructureConstants.Elevator.crossbarBottom.getInch())
				|| (GPelevator.getY().getInch() > SuperStructureConstants.Elevator.crossbarBottom.getInch()
						&& SPelevator.getY().getInch() < SuperStructureConstants.Elevator.crossbarBottom.getInch())
				|| (GPelevator.getY().getInch() < SuperStructureConstants.Elevator.crossbarBottom.plus(SuperStructureConstants.Elevator.crossbarWidth).getInch()
						&& GPelevator.getY().getInch() > SuperStructureConstants.Elevator.crossbarBottom.getInch()) && (

				// check if the elbow is in danger of hitting something, I don't care about the height as long as the intake isn't passed through right now

				goalState.getElbowAngle().getDegree() < -95 || currentState.getElbowAngle().getDegree() < -95

				)) {
			// I think this should be one of the first move commands, above anything else
			var doWeNeedToSafeElevatorFirst = (Util.max(GPelevator.getY(), SPelevator.getY()).getInch() < SuperStructureConstants.Elevator.minimumPassThroughAboveCrossbar.getInch());

			if (doWeNeedToSafeElevatorFirst)
				this.queue.addSequentialLoggable(new ElevatorMove(SuperStructureConstants.Elevator.minimumPassThroughAboveCrossbar));
			if (doWeNeedToSafeElevatorFirst)
				Logger.log("so if we try to yeet we will hit the crossbar. So let's yeet the elevator up to " + SuperStructureConstants.Elevator.minimumPassThroughAboveCrossbar.getInch());

			Logger.log("So apparently the elbow is in danger of hitting the crossbar. So uhh let's stow the arm.");
			this.queue.addSequentialLoggable(new ArmMove(SuperStructure.iPosition.STOWED));
		}

		boolean isLongClimb = Math.abs(goalState.getElevatorHeight().minus(currentState.getElevatorHeight()).getInch()) >= SuperStructureConstants.Elevator.kElevatorLongRaiseDistance.getInch();

		// if (isLongClimb) {
		// Logger.log("Stowing arm on long climb");
		// this.queue.addSequentialLoggable(new ArmWaitForElevator(SuperStructure.iPosition.STOWED, minUnCrashHeight.getHeight(), LengthKt.getInch(3)));
		// this.queue.addSequentialLoggable(new ArmMove(SuperStructure.iPosition.STOWED/*, minUnCrashHeight.getHeight(), LengthKt.getInch(3)*/));
		// }
		// figure out if the intake is gunna yeet itself into the bottom. This is done by finding the worst case (intake and stuff as far pitched down as possible)
		var worstCaseElbow = Util.getWorstCase(RoundRotation2d.getDegree(-90), goalState.getElbowAngle(), currentState.getElbowAngle());
		var worstCaseWrist = Util.getWorstCase(
				RoundRotation2d.getDegree(-90),
				getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()),
				getUnDumbWrist(currentState.getWristAngle(), currentState.getElbowAngle()));

		Logger.log(String.format("WORST CASE ELBOW: (%s) WORST CASE WRIST: (%s)", worstCaseElbow.toString(), worstCaseWrist.toString()));

		// now that we have a worst case angle, apply that to move the superstructure. 
		// First off, if moving the intake first would cause anything to hit the electronics, move the elevator up.
		// One exception: if the elbow is beyond the frame perimeter it's fine

		var rawGoalProximal = new Translation2d(SuperStructureConstants.Elbow.carriageToIntake, goalState.getElbowAngle().toRotation2d());
		var rawStartProximal = new Translation2d(SuperStructureConstants.Elbow.carriageToIntake, currentState.getElbowAngle().toRotation2d());

		// figure out the worst case of these angles, to find the _lowest_ point the intake can _ever_ be
		var worstCaseCarriageToEOI = new Translation2d(
				SuperStructureConstants.Elbow.carriageToIntake,
				worstCaseElbow.toRotation2d())
						.plus(
								new Translation2d(
										SuperStructureConstants.Wrist.intakeOut,
										worstCaseWrist.toRotation2d()));

		var isWithinFramePerimeter = (rawGoalProximal.getX().getInch() < SuperStructureConstants.kCarriageToFramePerimeter.getInch())
				|| (rawStartProximal.getX().getInch() < SuperStructureConstants.kCarriageToFramePerimeter.getInch());

		if (isWithinFramePerimeter)
			Logger.log("current or goal state is within the frame perimeter!");

		// first, check if trying to move the arms _right now_ would make something hit
		var worstCaseStartingPos = worstCaseCarriageToEOI.plus(new Translation2d(currentState.getElevator().height, LengthKt.getInch(0)));
		var worstCaseGoalPos = worstCaseCarriageToEOI.plus(new Translation2d(goalState.getElevator().height, LengthKt.getInch(0)));
		var minUnCrashHeight = new ElevatorState(worstCaseCarriageToEOI.getY().times(-1).plus(LengthKt.getInch(4)));

		if (worstCaseStartingPos.getY().getInch() < SuperStructureConstants.Elevator.electronicsHeight.getInch() || worstCaseGoalPos.getY().getInch() < SuperStructureConstants.Elevator.electronicsHeight.getInch()) {

			// TODO check if the end state is going to hit anything

			Logger.log("gunna slap the electronics plate, gotta move the elevator first");
			queue.addSequentialLoggable(new ElevatorMove(minUnCrashHeight));

		}

		// TODO where should passthrough go?
		// Logger.log("goal pos elbow end x: " + GPwrist.getX().getInch() + " startpoint pos elbow end: " + SPwrist.getX().getInch());
		// if (GPwrist.getX().getInch() > 8 && SPwrist.getX().getInch() < -8) {
		// queue.addSequentialLoggable(new PassThroughReverse());
		// } else if (GPwrist.getX().getInch() < -8 && SPwrist.getX().getInch() > 8) {
		// queue.addSequentialLoggable(new PassThroughForward());
		// }

		// Logger.log("pass");

		//CHECK the position of the intake -- hatch or cargo
		// IF it's a long climb

		// this.queue.addSequentialLoggable(new ArmWaitForElevator(goalState.getAngle(), goalState.getElevatorHeight(), startArmTol.plus(LengthKt.getInch(5)),
		// goalState.getElevatorHeight().getInch() < currentState.getElevatorHeight().getInch()));

		// ok so by now the elevator should be such that we can safely move stuff?
		// TODO make both move at same time if safe?
		Logger.log("Moving arm to final state");
		queue.addSequentialLoggable(new ArmMove(goalState.getAngle(), "final arm move"));

		Logger.log("moving elevator to final state");
		this.queue.addSequentialLoggable(new ElevatorMove(goalState.getElevator()));

		return true;
	}

	public TestableCommandGroup getQueue() {
		System.out.println("queue gotten");
		return this.queue;
	}

	// @Override
	protected void initialize() {
		// queue.start();
		// var current = SuperStructure.getInstance().updateState();
		// var current = new SuperStructureState(new ElevatorState(LengthKt.getInch(3.5)), iPosition.CARGO_GRAB);
		// var current = iPosition.HATCH_GRAB_INSIDE;

	}

	// @Override
	protected boolean isFinished() {
		// return queue.isCompleted();
		return true;
	}

	abstract class AbstractCommand {
		String name;

		public void setName(String name) {
			this.name = name;
		}
	}

	class ElevatorMove extends AbstractCommand {
		ElevatorState target;

		public ElevatorMove(ElevatorState target, String name) {
			this.target = target;
			super.setName(name);
		}

		public ElevatorMove(ElevatorState target) {
			this(target, target.toString());
		}

		public ElevatorMove(Length setpoint) {
			this(new ElevatorState(setpoint));
		}

		@Override
		public String toString() {
			return "ElevatorMove " + target.toString();
		}
	}

	class ArmMove extends AbstractCommand {
		public IntakeAngle angle;

		public ArmMove(IntakeAngle target) {
			this.angle = target;
			this.name = "ArmMove to " + target.toString();
		}

		public ArmMove(IntakeAngle target, String name) {
			this.name = name;
			this.angle = target;
		}

		@Override
		public String toString() {
			return "ArmMove to " + angle.toString();
		}
	}

	class TestableCommandGroup {
		ArrayList<String> commandLog;

		public ArrayList<String> getCommandLog() {
			return commandLog;
		}

		public TestableCommandGroup() {
			this.commandLog = new ArrayList<>();
		}

		public void addSequentialLoggable(AbstractCommand command) {
			Logger.log("Commamd " + command.name + " added in sequential mode");//
			commandLog.add("Commamd " + command.name + " added in sequential mode");
		}

		public void addParallelLoggable(AbstractCommand command) {
			Logger.log("Commamd " + command.name + " added in parallel mode");
			commandLog.add("Commamd " + command.name + " added in parallel mode");// and does \t" + command.toString());
		}
	}

}
