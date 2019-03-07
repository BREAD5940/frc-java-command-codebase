package frc.robot.planners;

import java.util.Arrays;
import java.util.Optional;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.SuperStructureConstants;
import frc.robot.SuperStructureConstants.Wrist;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.routines.passthrough.PassThroughForward;
import frc.robot.commands.auto.routines.passthrough.PassThroughReverse;
import frc.robot.commands.subsystems.superstructure.ArmMove;
import frc.robot.commands.subsystems.superstructure.ArmWaitForElevator;
import frc.robot.commands.subsystems.superstructure.ElevatorMove;
import frc.robot.lib.Logger;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

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
public class SuperstructureMotion extends Command {
	/* RELEVANT COMMANDS:
	  - ElevatorMove
	  - ArmMove
	  - ArmWaitForElevator
	 */

	private SuperstructureMotion() {
		requires(SuperStructure.getInstance());

		requires(SuperStructure.getInstance().getWrist());
		requires(SuperStructure.getInstance().getElbow());
		requires(SuperStructure.getElevator());
	}

	public SuperstructureMotion(SuperStructureState gsIn) {
		plan(gsIn, SuperStructure.getInstance().getCurrentState());

		requires(SuperStructure.getInstance().getWrist());
		requires(SuperStructure.getInstance().getElbow());
		requires(SuperStructure.getElevator());
	}

	private static SuperstructureMotion instance_;
	protected AutoCommandGroup queue = new AutoCommandGroup();
	// protected CommandGroup eleQueue = new CommandGroup();
	// protected CommandGroup armQueue = new CommandGroup();
	protected Optional<Command> current;

	public static SuperstructureMotion getInstance() {
		if (instance_ == null) {
			instance_ = new SuperstructureMotion();
		}
		return instance_;
	}

	public boolean plan(SuperStructureState gsIn, SuperStructureState currentState) {
		var goalState = new SuperStructureState(gsIn);
		Length startArmTol = LengthKt.getInch(3);
		//CHECK if the current and goal match
		if (goalState.isEqualTo(currentState)) {
			Logger.log("Goal and current states same.");
			return true;
		}
		//SAFE illegal inputs
		if (goalState.getElevatorHeight().getInch() > SuperStructureConstants.Elevator.top.getInch() - SuperStructureConstants.Elevator.carriageHeight.getInch()) {
			Logger.log("Elevator high");
			goalState.getElevator().setHeight(SuperStructureConstants.Elevator.top); // constrain elevator to max height
		} else if (goalState.getElevatorHeight().getInch() < SuperStructureConstants.Elevator.bottom.getInch()) {
			Logger.log("Elevator low");
			goalState.getElevator().setHeight(SuperStructureConstants.Elevator.bottom); // constrain elevator to min height
		}

		if (goalState.getElbowAngle().getDegree() > SuperStructureConstants.Elbow.kElbowMax.getDegree()) {
			Logger.log("Elbow big");
			goalState.getElbow().setAngle(SuperStructureConstants.Elbow.kElbowMax); // Constrain elbow to max
		} else if (goalState.getElbowAngle().getDegree() < SuperStructureConstants.Elbow.kElbowMin.getDegree()) {
			Logger.log("Elbow small");
			goalState.getElbow().setAngle(SuperStructureConstants.Elbow.kElbowMin); // Constrain elbow to min
		}

		// TODO safe the wrist, which is stupid and changes a lot. Maybe we need a equation or something for it?

		//DEFINE the three goal points -- elevator, wrist, and end of intake
		Translation2d GPelevator = new Translation2d(LengthKt.getInch(0), goalState.getElevatorHeight()); // TODO maybe change constructor to use a Translation2d fromed from a Length and Rotation2d?
		Translation2d GPwrist = new Translation2d(SuperStructureConstants.Elbow.carriageToIntake, goalState.getElbowAngle().toRotation2d()).plus(GPelevator);

		Translation2d GPeoi = new Translation2d(LengthKt.getInch(goalState.getWristAngle().getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()),
				LengthKt.getInch(goalState.getWristAngle().getSin() * SuperStructureConstants.Wrist.intakeOut.getInch())).plus(GPwrist);

		//DEFINE the three start points -- elevator, wrist, and end of intake
		Translation2d SPelevator = new Translation2d(LengthKt.getInch(0), currentState.getElevatorHeight());
		Translation2d SPwrist = new Translation2d(LengthKt.getInch(currentState.getElbowAngle().getCos() * SuperStructureConstants.Elbow.carriageToIntake.getInch()),
				LengthKt.getInch(currentState.getElbowAngle().getSin() * SuperStructureConstants.Elbow.carriageToIntake.getInch())).plus(SPelevator);
		Translation2d SPeoi = new Translation2d(LengthKt.getInch(currentState.getWristAngle().getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()),
				LengthKt.getInch(currentState.getWristAngle().getSin() * SuperStructureConstants.Wrist.intakeOut.getInch())).plus(SPwrist);

		// FIXME so the issue here is that the maximum position of the wrist depends on the proximal (elbow) angle. So we have to measure it somehow yay. Also the sprocket on there means that the wrist will slowly rotate as the proximal joint rotates
		//FIXME mostly fixed, check math
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
			GPeoi = new Translation2d(LengthKt.getInch(goalState.getWristAngle().getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getX()),
					LengthKt.getInch(goalState.getWristAngle().getSin() * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getY()));
		}

		if (GPeoi.getY().minus(Wrist.intakeAbove).getInch() < SuperStructureConstants.Elevator.electronicsHeight.getInch()) {
			Logger.log("intake still too low");
			RoundRotation2d tempTheta = goalState.getWristAngle();
			tempTheta = RoundRotation2d.getRadian(
					Math.asin(
							(GPeoi.getY().plus(Wrist.intakeAbove).getInch() - GPwrist.getY().getInch())
									/ SuperStructureConstants.Wrist.intakeOut.getInch()));
			goalState.getWrist().setAngle(tempTheta);
			GPeoi = new Translation2d(LengthKt.getInch(tempTheta.getCos() * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getX()),
					LengthKt.getInch(Math.sin(tempTheta.getRadian()) * SuperStructureConstants.Wrist.intakeOut.getInch()).plus(GPwrist.getY()));
		}

		//FIND the lowest goal and end points
		Translation2d lowestGP = GPeoi;
		for (Translation2d current : Arrays.asList(GPelevator, GPwrist, GPeoi, 
					GPeoi.minus(new Translation2d(LengthKt.getInch(0), SuperStructureConstants.Wrist.intakeAbove)))){
			lowestGP = (lowestGP.getY().getInch() >= current.getY().getInch()) ? current : lowestGP;
		}

		Translation2d lowestSP = SPeoi;
		for (Translation2d current : Arrays.asList(SPelevator, SPwrist, SPeoi,
					SPeoi.minus(new Translation2d(LengthKt.getInch(0), SuperStructureConstants.Wrist.intakeAbove)))){
			lowestSP = (lowestSP.getY().getInch() >= current.getY().getInch()) ? current : lowestSP;
		}

		//SAFE potential crashes IN BETWEEN states
		if(lowestGP.getY().getInch() < GPelevator.getY().getInch()){
			//FIND how much of the intake is in the way

			//SET the tolerance to that number

		}

		startArmTol = GPelevator.getY().minus(LengthKt.getInch(Math.abs(lowestSP.getY().minus(SPelevator.getY()).getInch())));

		startArmTol = (startArmTol.getInch() > (Math.abs(GPelevator.getY().getInch() - Math.abs(lowestGP.getY().minus(GPelevator.getY()).getInch()))))
										? startArmTol : (LengthKt.getInch(Math.abs(GPelevator.getY().getInch() - Math.abs(lowestGP.getY().minus(GPelevator.getY()).getInch()))));


		//CLEAR the queue
		queue = new AutoCommandGroup();

		if (GPwrist.getX().getInch() > 0 && SPwrist.getX().getInch() < 0) {
			queue.addSequential(new PassThroughReverse());
		} else if (GPwrist.getX().getInch() < 0 && SPwrist.getX().getInch() > 0) {
			queue.addSequential(new PassThroughForward());
		}

		//CHECK the position of the intake -- hatch or cargo
		// IF it's a long climb
		boolean isLongClimb = Math.abs(goalState.getElevatorHeight().minus(currentState.getElevatorHeight()).getInch()) >= SuperStructureConstants.Elevator.kElevatorLongRaiseDistance.getInch();

		if (isLongClimb) {
			queue.addParallel(new ArmMove(SuperStructure.iPosition.STOWED));
		}

		//CHECK if the elevator point is in proximity to the crossbar - if it is, stow it
		if ((GPelevator.getY().getInch() < SuperStructureConstants.Elevator.crossbarBottom.getInch()
				&& SPelevator.getY().getInch() > SuperStructureConstants.Elevator.crossbarBottom.getInch())
				|| (GPelevator.getY().getInch() > SuperStructureConstants.Elevator.crossbarBottom.getInch()
						&& SPelevator.getY().getInch() < SuperStructureConstants.Elevator.crossbarBottom.getInch())
				|| (GPelevator.getY().getInch() < SuperStructureConstants.Elevator.crossbarBottom.plus(SuperStructureConstants.Elevator.crossbarWidth).getInch()
						&& GPelevator.getY().getInch() > SuperStructureConstants.Elevator.crossbarBottom.getInch())) {
			queue.addSequential(new ArmMove(SuperStructure.iPosition.STOWED));
		}

		queue.addParallel(new ArmWaitForElevator(goalState.getAngle(), goalState.getElevatorHeight(), startArmTol,
				goalState.getElevatorHeight().getInch() < currentState.getElevatorHeight().getInch()));
		queue.addSequential(new ElevatorMove(goalState.getElevator()));

		return true;
	}

	public AutoCommandGroup getQueue() {
		return queue;
	}

	@Override
	protected void initialize() {
		queue.start();
	}

	@Override
	protected boolean isFinished() {
		return queue.isCompleted();
	}

}
