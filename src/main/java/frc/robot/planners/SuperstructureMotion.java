package frc.robot.planners;

import java.util.Arrays;
import java.util.Optional;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.SILengthConstants;
import org.team5940.pantry.exparimental.command.Command;
import org.team5940.pantry.exparimental.command.InstantCommand;
import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.SuperStructureConstants;
import frc.robot.commands.subsystems.superstructure.ArmMove;
import frc.robot.commands.subsystems.superstructure.ElevatorMove;
import frc.robot.lib.Logger;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * Instructions for using this mutant command-thing:
 *  - Do NOT call the constructor (unless ur OI)
 *  - Call the planner
 *  - Call SuperstructureMotion.getInstance().start();
 *    - This will iterate through the planned commandQueue
 *    - It'll end when it's done
 * 
 * @author Jocelyn McHugo
 */
public class SuperstructureMotion extends SendableCommandBase {
	/* RELEVANT COMMANDS:
	  - ElevatorMove
	  - ArmMove
	  - ArmWaitForElevator
	 */

	boolean isReal = false;
	private SuperStructureState gsIn;

	@Deprecated
	private SuperstructureMotion() {
		addRequirements(SuperStructure.getInstance());
		addRequirements(SuperStructure.getInstance().getWrist());
		addRequirements(SuperStructure.getInstance().getElbow());
		addRequirements(SuperStructure.getElevator());
	}

	public SuperstructureMotion(SuperStructureState gsIn, SuperStructureState current) {
		System.out.println("ssmotion instan");
		this.gsIn = gsIn;
		addRequirements(SuperStructure.getInstance().getWrist());
		addRequirements(SuperStructure.getInstance().getElbow());
		addRequirements(SuperStructure.getElevator());
		addRequirements(SuperStructure.getInstance());
	}

	public SuperstructureMotion(SuperStructureState gsIn) {
		this(gsIn, SuperStructure.getInstance().updateState());
	}

	private static SuperstructureMotion instance_;
	protected Command queue = new InstantCommand();
	// protected CommandGroup eleQueue = new CommandGroup();
	// protected CommandGroup armQueue = new CommandGroup();
	protected Optional<Command> current;

	@Deprecated
	public static SuperstructureMotion getInstance() {
		if (instance_ == null) {
			instance_ = new SuperstructureMotion();
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

		Logger.log("Goal position: " + goalState.toCSV() + String.format("gpWrist: x %s y %s", GPwrist.getX() / SILengthConstants.kInchToMeter, GPwrist.getY() / SILengthConstants.kInchToMeter));

		/** 
		 * goal point end of intake 
		 */
		Translation2d GPeoi = new Translation2d(
				LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()),
				LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getSin() * SuperStructureConstants.Wrist.intakeOut.getInch())).plus(GPwrist);

		//DEFINE the three start points -- elevator, wrist, and end of intake
		Translation2d SPelevator = new Translation2d(LengthKt.getInch(0), currentState.getElevatorHeight());
		// Translation2d SPwrist = new Translation2d(LengthKt.getInch(currentState.getElbowAngle().getCos() * SuperStructureConstants.Elbow.carriageToIntake / SILengthConstants.kInchToMeter),
		// LengthKt.getInch(currentState.getElbowAngle().getSin() * SuperStructureConstants.Elbow.carriageToIntake / SILengthConstants.kInchToMeter)).plus(SPelevator);

		Translation2d SPwrist = new Translation2d(SuperStructureConstants.Elbow.carriageToIntake, currentState.getElbowAngle().toRotation2d()).plus(SPelevator);

		Translation2d SPeoi = new Translation2d(
				LengthKt.getInch(getUnDumbWrist(currentState.getWristAngle(), currentState.getElbowAngle()).getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()),
				LengthKt.getInch(getUnDumbWrist(currentState.getWristAngle(), currentState.getElbowAngle()).getSin() * SuperStructureConstants.Wrist.intakeOut.getInch())).plus(SPwrist);

		Logger.log("made points");

		if (Math.atan((GPeoi.getY() / SILengthConstants.kInchToMeter + GPwrist.getY() / SILengthConstants.kInchToMeter) / (GPeoi.getX() / SILengthConstants.kInchToMeter + GPwrist.getX() / SILengthConstants.kInchToMeter)) > SuperStructureConstants.Wrist.kWristMax.getRadian()) {
			Logger.log("Wrist big");
			goalState.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMax); // constrain wrist to max
		} else if (Math.atan((GPeoi.getY() / SILengthConstants.kInchToMeter + GPwrist.getY() / SILengthConstants.kInchToMeter) / (GPeoi.getX() / SILengthConstants.kInchToMeter + GPwrist.getX() / SILengthConstants.kInchToMeter)) < SuperStructureConstants.Wrist.kWristMin.getRadian()) {
			Logger.log("Wrist small");
			goalState.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMin); // constrain wrist to min
		}

		//SAFE potential crashes on the end state (FIXME not necessarily in between states?)
		if (GPwrist.getY() / SILengthConstants.kInchToMeter < SuperStructureConstants.Elevator.electronicsHeight.getInch() || GPeoi.getY() / SILengthConstants.kInchToMeter < SuperStructureConstants.Elevator.electronicsHeight.getInch()) {
			Logger.log("intake too low");
			RoundRotation2d tempTheta = goalState.getElbowAngle();
			tempTheta = RoundRotation2d.getRadian(
					Math.asin(
							Math.abs(SuperStructureConstants.Elevator.electronicsHeight.getMeter() - GPelevator.getY())
									/ SuperStructureConstants.Elbow.carriageToIntake.getMeter()));
			goalState.getElbow().setAngle(tempTheta);
			GPwrist = new Translation2d(LengthKt.getInch(tempTheta.getCos() * SuperStructureConstants.Elbow.carriageToIntake.getInch()),
					LengthKt.getInch(Math.sin(tempTheta.getRadian()) * SuperStructureConstants.Elbow.carriageToIntake.getInch() / SILengthConstants.kInchToMeter).plus(LengthKt.getMeter(GPelevator.getY())));

			GPeoi = new Translation2d(LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(LengthKt.getMeter(GPelevator.getY())),
					LengthKt.getInch(getUnDumbWrist(goalState.getWristAngle(), goalState.getElbowAngle()).getSin() * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(LengthKt.getMeter(GPelevator.getY())));
		}

		Logger.log("a r c s i n");

		//FIND the lowest goal and end points
		Translation2d lowestGP = GPeoi;
		for (Translation2d current : Arrays.asList(GPelevator, GPwrist, GPeoi,
				GPeoi.minus(new Translation2d(LengthKt.getInch(0), SuperStructureConstants.Wrist.intakeAbove)))) {
			lowestGP = (lowestGP.getY() / SILengthConstants.kInchToMeter >= current.getY() / SILengthConstants.kInchToMeter) ? current : lowestGP;
		}

		Translation2d lowestSP = SPeoi;
		for (Translation2d current : Arrays.asList(SPelevator, SPwrist, SPeoi,
				SPeoi.minus(new Translation2d(LengthKt.getInch(0), SuperStructureConstants.Wrist.intakeAbove)))) {
			lowestSP = (lowestSP.getY() / SILengthConstants.kInchToMeter >= current.getY() / SILengthConstants.kInchToMeter) ? current : lowestSP;
		}

		Logger.log("lowests");

		//SAFE potential crashes IN BETWEEN states
		if (lowestGP.getY() / SILengthConstants.kInchToMeter < GPelevator.getY() / SILengthConstants.kInchToMeter) {
			//FIND how much of the intake is in the way

			//SET the tolerance to that number

		}

		startArmTol = LengthKt.getMeter(GPelevator.getY()).minus(LengthKt.getInch(Math.abs(LengthKt.getMeter(lowestSP.getY()).minus(LengthKt.getMeter(SPelevator.getY())).getInch())));

		startArmTol = (startArmTol.getInch() > (Math.abs(GPelevator.getY() / SILengthConstants.kInchToMeter - Math.abs(lowestGP.getY() - (GPelevator.getY()) / SILengthConstants.kInchToMeter))))
				? startArmTol
				: (LengthKt.getInch(Math.abs(GPelevator.getY() / SILengthConstants.kInchToMeter - Math.abs(lowestGP.getY() - (GPelevator.getY()) / SILengthConstants.kInchToMeter))));
		Logger.log("tolerances");
		//CLEAR the queue
		this.queue = new InstantCommand();
		Logger.log("queue cleared");

		//CHECK if the elevator point is in proximity to the crossbar - if it is, stow it
		// This is the VERY FIRST thing we do so that we make sure that we don't slap a meme
		if ((GPelevator.getY() / SILengthConstants.kInchToMeter < SuperStructureConstants.Elevator.crossbarBottom.getInch()
				&& SPelevator.getY() / SILengthConstants.kInchToMeter > SuperStructureConstants.Elevator.crossbarBottom.getInch())
				|| (GPelevator.getY() / SILengthConstants.kInchToMeter > SuperStructureConstants.Elevator.crossbarBottom.getInch()
						&& SPelevator.getY() / SILengthConstants.kInchToMeter < SuperStructureConstants.Elevator.crossbarBottom.getInch())
				|| (GPelevator.getY() / SILengthConstants.kInchToMeter < SuperStructureConstants.Elevator.crossbarBottom.plus(SuperStructureConstants.Elevator.crossbarWidth).getInch()
						&& GPelevator.getY() / SILengthConstants.kInchToMeter > SuperStructureConstants.Elevator.crossbarBottom.getInch()) && (

				// check if the elbow is in danger of hitting something, I don't care about the height as long as the intake isn't passed through right now
				goalState.getElbowAngle().getDegree() < -95 || currentState.getElbowAngle().getDegree() < -95

				)) {
			// I think this should be one of the first move commands, above anything else

			var doWeNeedToSafeElevatorFirst = (Math.max(GPelevator.getY(), SPelevator.getY()) / SILengthConstants.kInchToMeter < SuperStructureConstants.Elevator.minimumPassThroughAboveCrossbar.getInch());

			if (doWeNeedToSafeElevatorFirst)
				this.queue = this.queue.andThen(new ElevatorMove(SuperStructureConstants.Elevator.minimumPassThroughAboveCrossbar));
			this.queue = queue.andThen(new ArmMove(SuperStructure.iPosition.STOWED));
		}

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

		var isWithinFramePerimeter = (rawGoalProximal.getX() / SILengthConstants.kInchToMeter < SuperStructureConstants.kCarriageToFramePerimeter.getInch())
				|| (rawStartProximal.getX() / SILengthConstants.kInchToMeter < SuperStructureConstants.kCarriageToFramePerimeter.getInch());

		if (isWithinFramePerimeter)
			Logger.log("current or goal state is within the frame perimeter!");

		// first, check if trying to move the arms _right now_ would make something hit
		var worstCaseStartingPos = worstCaseCarriageToEOI.plus(new Translation2d(currentState.getElevator().height, LengthKt.getInch(0)));
		var worstCaseGoalPos = worstCaseCarriageToEOI.plus(new Translation2d(goalState.getElevator().height, LengthKt.getInch(0)));
		var minUnCrashHeight = new ElevatorState(worstCaseCarriageToEOI.getY() * (-1) + (LengthKt.getInch(4)).getMeter());

		if (worstCaseStartingPos.getY() / SILengthConstants.kInchToMeter < SuperStructureConstants.Elevator.electronicsHeight.getInch() || worstCaseGoalPos.getY() / SILengthConstants.kInchToMeter < SuperStructureConstants.Elevator.electronicsHeight.getInch()) {
			Logger.log("gunna slap the electronics plate, gotta move the elevator first");

			// TODO check if the end state is going to hit anything

			queue = queue.andThen(new ElevatorMove(minUnCrashHeight));

		}

		// TODO where should passthrough go?
		// Logger.log("goal pos elbow end x: " + GPwrist.getX() / SILengthConstants.kInchToMeter + " startpoint pos elbow end: " + SPwrist.getX() / SILengthConstants.kInchToMeter);
		// if (GPwrist.getX() / SILengthConstants.kInchToMeter > 8 && SPwrist.getX() / SILengthConstants.kInchToMeter < -8) {
		// queue.addSequentialLoggable(new PassThroughReverse(), isReal);
		// } else if (GPwrist.getX() / SILengthConstants.kInchToMeter < -8 && SPwrist.getX() / SILengthConstants.kInchToMeter > 8) {
		// queue.addSequentialLoggable(new PassThroughForward(), isReal);
		// }

		// Logger.log("pass");

		//CHECK the position of the intake -- hatch or cargo
		// IF it's a long climb

		boolean isLongClimb = Math.abs(goalState.getElevatorHeight().minus(currentState.getElevatorHeight()).getInch()) >= SuperStructureConstants.Elevator.kElevatorLongRaiseDistance.getInch();

		if (isLongClimb) {
			// this.queue.addSequentialLoggable(new ArmWaitForElevator(SuperStructure.iPosition.STOWED, minUnCrashHeight.getHeight(), LengthKt.getInch(3)), isReal);
			queue = queue.andThen(new ArmMove(SuperStructure.iPosition.STOWED/*, minUnCrashHeight.getHeight(), LengthKt.getInch(3)*/));
		}

		// this.queue.addSequentialLoggable(new ArmWaitForElevator(goalState.getAngle(), goalState.getElevatorHeight(), startArmTol.plus(LengthKt.getInch(5)),
		// goalState.getElevatorHeight() / SILengthConstants.kInchToMeter < currentState.getElevatorHeight() / SILengthConstants.kInchToMeter), isReal);

		// ok so by now the elevator should be such that we can safely move stuff?
		queue = queue.andThen(new ArmMove(goalState.getAngle()));

		queue = queue.andThen(new ElevatorMove(goalState.getElevator()));

		return true;
	}

	public Command getQueue() {
		System.out.println("queue gotten");
		return this.queue;
	}

	@Override
	public void initialize() {
		// queue.start();
		// var current = SuperStructure.getInstance().updateState();
		var current = new SuperStructureState(new ElevatorState(LengthKt.getInch(3.5)), iPosition.CARGO_GRAB);
		plan(this.gsIn, current);
		System.out.println("===================================================================");
		Logger.log(String.format("Start state (%s) Goal state (%s)", current.toString(), gsIn.toString()));
		// Logger.log(getQueue().getCommandLog().get(0));

		System.out.println("===================================================================");

	}

	@Override
	public boolean isFinished() {
		// return queue.isCompleted();
		return true;
	}

}
