package frc.robot.commands.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperStructure.getDumbWrist;

import java.util.function.Supplier;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.superstructure.SuperStructure;

public class PassThrough extends ConditionalCommand {

	private static final double kProximalMaxVel = 190d / 360d * 2 * Math.PI;
	private static final double kWristMaxVel = 190d / 360d * 2 * Math.PI;

	public static class SyncedMove extends Command {

		private final double goal;
		private final double goalWrist;
		private final RoundRotation2d goalRotation2d;
		private final RoundRotation2d goalWristRotation2d;
		private final boolean isFrontToBack;
		private final SuperStructure structure;
		private RoundRotation2d lastCommandedProximal;
		private final double proximalVelocity;
		private double lastTime = 0;
		private boolean moveIteratorFinished = false;
		private SuperStructureState lastObservedState = new SuperStructureState();

		public SyncedMove(double goalAngle, double proximalMaxVel, double wristMaxVel, boolean isFrontToBack, SuperStructure structure) {
			this.goal = goalAngle + (isFrontToBack ? -30 : 0);
			this.goalWrist = goalAngle;
			this.goalWristRotation2d = RoundRotation2d.getRadian(goalWrist);
			this.goalRotation2d = RoundRotation2d.getRadian(goal);
			this.structure = structure;
			this.isFrontToBack = isFrontToBack;
			this.proximalVelocity = Math.abs(Math.min(Math.abs(proximalMaxVel), Math.abs(wristMaxVel))) * 0.8 * (isFrontToBack ? -1 : 1);
		}

		public SyncedMove(double goalAngle, boolean isFrontToBack, SuperStructure structure) {
			this(goalAngle, kProximalMaxVel, kWristMaxVel, isFrontToBack, structure);
		}

		@Override
		public void initialize() {
			lastCommandedProximal = structure.getCurrentState().getElbowAngle();
			lastTime = Timer.getFPGATimestamp();
			moveIteratorFinished = false;

			System.out.println("initial proximal: " + lastCommandedProximal);
		}

		@Override
		public void execute() {

			System.out.println("MOVE I9TERATOR " + moveIteratorFinished);

			if (moveIteratorFinished)
				return;

			var now = Timer.getFPGATimestamp();
			var dt = now - lastTime;
			var currentState = structure.getCurrentState();
			this.lastObservedState = currentState;

			RoundRotation2d nextProximal = lastCommandedProximal;

			if (Math.abs(SuperStructure.getUnDumbWrist(lastObservedState.getWristAngle(), lastObservedState.getElbowAngle()).getDegree() - lastCommandedProximal.getDegree()) < 50) {
				nextProximal = this.lastCommandedProximal.plus(RoundRotation2d.getRadian(proximalVelocity * dt));
			}

			if (nextProximal.getDegree() < -205) {
				nextProximal = RoundRotation2d.getDegree(-205);
				moveIteratorFinished = true;
				System.out.println("SETTING MOVE ITERATOR TO TRUE 1");
			} else if (nextProximal.getDegree() > 5) {
				nextProximal = RoundRotation2d.getDegree(5);
				moveIteratorFinished = true;
				System.out.println("SETTING MOVE ITERATOR TO TRUE 2");
			}

			System.out.println("next proximal: " + nextProximal.getDegree());

			if (isFrontToBack) {
				if (nextProximal.getRadian() < goal)
					nextProximal = goalRotation2d;
			} else {
				if (nextProximal.getRadian() > goal)
					nextProximal = goalRotation2d;
			}

			if (isFrontToBack) {
				if (nextProximal.getRadian() < goal) {
					this.moveIteratorFinished = true;
					System.out.println("SETTING MOVE ITERATOR TO TRUE 3");
				}
			} else if (nextProximal.getRadian() > goal) {
				this.moveIteratorFinished = true;
				System.out.println("SETTING MOVE ITERATOR TO TRUE 4");
			}

			var nextWrist = getDumbWrist(currentState.getElbowAngle(), currentState.getElbowAngle());

			System.out.println("next elbow " + nextProximal);
			System.out.println("next wrist " + nextWrist);

			structure.getWrist().requestAngle(nextWrist);
			structure.getElbow().requestAngle(nextProximal);

			this.lastCommandedProximal = nextProximal;
			lastTime = now;

			System.out.println("====================");

			System.out.println("last observed prox " + lastObservedState.getElbowAngle().getDegree() + " last wrist " +
					lastObservedState.getWristAngle().getDegree() + " next elbow " + nextProximal +
					" next wrist " + nextWrist);

			System.out.println("====================");
		}

		@Override
		public boolean isFinished() {
			var toReturn = (structure.getWrist().isWithinTolerance(RoundRotation2d.getDegree(5),
					getDumbWrist(RoundRotation2d.getRadian(goal), lastObservedState.getElbowAngle()))
					&& structure.getElbow()
							.isWithinTolerance(RoundRotation2d.getDegree(5), RoundRotation2d.getRadian(goal))
					&& moveIteratorFinished) || moveIteratorFinished;

			System.out.println("pass thru done? " + toReturn);

			System.out.println("moveIteratorFinished? " + moveIteratorFinished);

			return toReturn;
		}
	}

	private final SuperStructure structure;

	public PassThrough(SuperStructure structure, Supplier<Boolean> isFrontToBack) {

		// super(setupConstructor(structure), isFrontToBack);

		super(new FrontToBack(structure), new BackToFront(structure));

		this.structure = structure;
		this.isFrontToBack = isFrontToBack;

	}

	private Supplier<Boolean> isFrontToBack;

	@Override
	protected boolean condition() {
		return isFrontToBack.get();
	}

	public static class FrontToBack extends CommandGroup {

		public FrontToBack(SuperStructure structure) {

			setInterruptible(false);

			addSequential(new PrintCommand("passing thru front to back"));
			addSequential(new SetHatchMech(Intake.HatchMechState.kClamped));
			//			addSequential(new LineEverythingUp(() -> SuperStructure.getInstance().getCurrentState().getElbowAngle()));
			addSequential(new ElevatorMove(LengthKt.getInch(22.5))); //todo check height
			addSequential(new SyncedMove(Math.toRadians(-160), true, structure));
			//-188 elbow -106 wrist
			addSequential(new ArmMove(new IntakeAngle(
					RoundRotation2d.getDegree(-193),
					RoundRotation2d.getDegree(-112))));
			addSequential(new ElevatorMove(LengthKt.getInch(5))); //todo check height
		}
	}

	public static class BackToFront extends CommandGroup {

		public BackToFront(SuperStructure structure) {

			setInterruptible(false);

			addSequential(new PrintCommand("passing thru back to front"));
			addSequential(new SetHatchMech(Intake.HatchMechState.kClamped));
			addSequential(new ElevatorMove(LengthKt.getInch(22.5))); //todo check height
			//			addSequential(new SyncedMove(Math.toRadians(-58), false, structure));
			addSequential(new SyncedMove(Math.toRadians(0), false, structure));
			addSequential(new ArmMove(SuperStructure.iPosition.HATCH_GRAB_INSIDE.getAngle()));
			addSequential(new ElevatorMove(SuperStructure.iPosition.HATCH_GRAB_INSIDE.getElevator()));

		}
	}

	//	public static class LineEverythingUp extends Command {
	//
	//		private final Supplier<RoundRotation2d> goalSupplier;
	//		private RoundRotation2d goalState = null;
	//		private boolean started = false;
	//		private SuperStructureState lastObservedState;
	//
	//		public LineEverythingUp(Supplier<RoundRotation2d> goal) {
	//			this.goalSupplier = goal;
	//		}
	//
	//		@Override
	//		protected void initialize() {
	//			this.lastObservedState = SuperStructure.getInstance().getCurrentState();
	//			started = true;
	//			goalState = goalSupplier.get();
	//		}
	//
	//		@Override
	//		protected void execute() {
	//			var structure = SuperStructure.getInstance();
	//			structure.getWrist().requestAngle(goalState);
	//			structure.getElbow().requestAngle(SuperStructure.getDumbWrist(goalState, goalState));
	//		}
	//
	//		@Override
	//		public boolean isFinished() {
	//
	//			var structure = SuperStructure.getInstance();
	//
	//			if(started == false && started) return true;
	//			if(started == false && !started) return false;
	//			var toReturn = (SuperStructure.getInstance().getWrist().isWithinTolerance(RoundRotation2d.getDegree(4),
	//					getDumbWrist(goalState, lastObservedState.getElbowAngle()))
	//					&& structure.getElbow().isWithinTolerance(RoundRotation2d.getDegree(4), goalState));
	//
	//
	//			System.out.println("move done? " + toReturn);
	//
	//			return toReturn;
	//		}
	//	}

	// public static Map<Boolean, Command> setupConstructor(SuperStructure structure) {

	// 	// create possible command groups
	// 	var frontToBackGroup = new CommandGroup();

	// 	frontToBackGroup.addSequential(new ElevatorMove(LengthKt.getInch(30))); //todo check height
	// 	frontToBackGroup.addSequential(new SyncedMove(Math.toRadians(-180), true, structure));

	// 	var backToFrontGroup = new CommandGroup();

	// 	backToFrontGroup.addSequential(new ElevatorMove(LengthKt.getInch(30))); //todo check height
	// 	backToFrontGroup.addSequential(new SyncedMove(Math.toRadians(180), false, structure));

	// 	var map = new HashMap<Boolean, Command>();
	// 	map.put(true, frontToBackGroup);
	// 	map.put(false, backToFrontGroup);

	// 	return map;
	// }

}
