package frc.robot.commands.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperStructure.getDumbWrist;

import java.util.function.BooleanSupplier;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.team5940.pantry.exparimental.command.Command;
import org.team5940.pantry.exparimental.command.ConditionalCommand;
import org.team5940.pantry.exparimental.command.SendableCommandBase;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class PassThrough extends ConditionalCommand {

	private static final double kProximalMaxVel = 150d / 360d * 2 * Math.PI;
	private static final double kWristMaxVel = 150d / 360d * 2 * Math.PI;

	public static class SyncedMove extends SendableCommandBase {

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
		}

		@Override
		public void execute() {

			if (moveIteratorFinished)
				return;

			var now = Timer.getFPGATimestamp();
			var dt = now - lastTime;
			var currentState = structure.getCurrentState();
			this.lastObservedState = currentState;

			var nextProximal = this.lastCommandedProximal.plus(RoundRotation2d.getRadian(proximalVelocity * dt));

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
				}
			} else if (nextProximal.getRadian() > goal) {
				this.moveIteratorFinished = true;
			}

			var nextWrist = getDumbWrist(currentState.getElbowAngle(), currentState.getElbowAngle());

			System.out.println("next elbow " + nextProximal);
			System.out.println("next wrist " + nextWrist);

			structure.getWrist().requestAngle(nextWrist);
			structure.getElbow().requestAngle(nextProximal);

			this.lastCommandedProximal = nextProximal;
			lastTime = now;
		}

		@Override
		public boolean isFinished() {
			return structure.getWrist().isWithinTolerance(RoundRotation2d.getDegree(5),
					getDumbWrist(RoundRotation2d.getRadian(goal), lastObservedState.getElbowAngle()))
					&& structure.getElbow()
							.isWithinTolerance(RoundRotation2d.getDegree(5), RoundRotation2d.getRadian(goal))
					&& moveIteratorFinished;
		}
	}

	private final SuperStructure structure;

	public PassThrough(SuperStructure structure, BooleanSupplier isFrontToBack) {

		// super(setupConstructor(structure), isFrontToBack);

		super(getPassThroughFrontToBack(structure), getPassThroughBackToFront(structure), isFrontToBack);

		this.structure = structure;
		this.isFrontToBack = isFrontToBack;

	}

	private BooleanSupplier isFrontToBack;

	private static Command getPassThroughFrontToBack(SuperStructure structure) {
		return (new ElevatorMove(LengthKt.getInch(30))) //todo check height
				.andThen(new SyncedMove(Math.toRadians(-180), true, structure));
	}

	private static Command getPassThroughBackToFront(SuperStructure structure) {
		return (new ElevatorMove(LengthKt.getInch(30))) //todo check height
				.andThen(new SyncedMove(Math.toRadians(180), false, structure));
	}

}
