package frc.robot.commands.subsystems.superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.team5940.pantry.experimental.command.Command;
import org.team5940.pantry.experimental.command.SelectCommand;
import org.team5940.pantry.experimental.command.SendableCommandBase;
import org.team5940.pantry.experimental.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class PassThrough extends SelectCommand<Boolean> {

	private static final double kProximalMaxVel = 100d / 360d * 2 * Math.PI;
	private static final double kWristMaxVel = 100d / 360d * 2 * Math.PI;

	private static RoundRotation2d getUnDumbWrist(RoundRotation2d dumbWrist, RoundRotation2d relevantProx) {
		var compensatedAngle = dumbWrist.plus(relevantProx.div(2));
		return compensatedAngle;
	}

	private static RoundRotation2d getDumbWrist(RoundRotation2d smartWrist, RoundRotation2d relevantProx) {
		var unCompensatedAngle = relevantProx.div(2).minus(smartWrist);
		return unCompensatedAngle;
	}

	private static class SyncedMove extends SendableCommandBase {

		private final double goal;
		private final RoundRotation2d goalRotation2d;
		private final boolean isFrontToBack;
		private final SuperStructure structure;
		private RoundRotation2d lastCommandedProximal;
		private final double proximalVelocity;
		private double lastTime = 0;
		private boolean moveIteratorFinished = false;
		private SuperStructureState lastObservedState = new SuperStructureState();

		private SyncedMove(double goalAngle, double proximalMaxVel, double wristMaxVel, boolean isFrontToBack, SuperStructure structure) {
			this.goal = goalAngle;
			this.structure = structure;
			this.isFrontToBack = isFrontToBack;
			this.proximalVelocity = Math.abs((proximalMaxVel < wristMaxVel) ? proximalMaxVel * 0.8 : wristMaxVel * 0.8) * (isFrontToBack ? -1 : 1);
			this.goalRotation2d = RoundRotation2d.getRadian(goal);
		}

		private SyncedMove(double goalAngle, boolean isFrontToBack, SuperStructure structure) {
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

			var nextProximal = this.lastCommandedProximal.plus(RoundRotation2d.getRadian(proximalVelocity / dt));

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

			var nextWrist = getDumbWrist(nextProximal, currentState.getElbowAngle());

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

	public PassThrough(SuperStructure structure, Supplier<Boolean> isFrontToBack) {

		super(setupConstructor(structure), isFrontToBack);

		this.structure = structure;

	}

	public static Map<Boolean, Command> setupConstructor(SuperStructure structure) {

		// create possible command groups
		var frontToBackGroup = new SequentialCommandGroup(
				new ElevatorMove(LengthKt.getInch(30)), //todo check height
				new SyncedMove(Math.toRadians(-180), true, structure));

		var backToFrontGroup = new SequentialCommandGroup(
				new ElevatorMove(LengthKt.getInch(30)), //todo check height
				new SyncedMove(Math.toRadians(180), false, structure));

		var map = new HashMap<Boolean, Command>();
		map.put(true, frontToBackGroup);
		map.put(false, backToFrontGroup);

		return map;
	}

}
