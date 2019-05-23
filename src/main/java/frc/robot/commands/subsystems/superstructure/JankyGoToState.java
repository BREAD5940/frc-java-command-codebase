package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class JankyGoToState extends CommandGroup {

	public JankyGoToState(Length height, IntakeAngle angles) {
		this(new SuperStructureState(new ElevatorState(height), angles));
	}

	public JankyGoToState(ElevatorState height, IntakeAngle angles) {
		this(new SuperStructureState(height, angles));
	}

	public JankyGoToState(SuperStructureState requ_) {

		requires(SuperStructure.getInstance());

		setInterruptible(false);

		addSequential(new PrintCommand("requested state: " + requ_.toCSV()));

		var safeMove = new ConditionalCommand(new ElevatorThanArm(requ_), new ArmThanElevator(requ_)) {

			@Override
			protected boolean condition() {
				var proximalThreshold = -18;
				var currentState = SuperStructure.getInstance().getCurrentState();
				var startAboveSafe = requ_.getElevatorHeight().getInch() > 25;
				var endAboveSafe = currentState.getElevatorHeight().getInch() > 25;
				var nowOutsideFrame = currentState.getElbowAngle().getDegree() > proximalThreshold;
				var willBeOutsideFrame = requ_.getElbowAngle().getDegree() > proximalThreshold;

				var shouldMoveElevatorFirst = (nowOutsideFrame && !willBeOutsideFrame && !startAboveSafe) || (nowOutsideFrame && willBeOutsideFrame) || ((-35 >= currentState.getElbowAngle().getDegree() && currentState.getElbowAngle().getDegree() >= -90) && (-50 >= requ_.getElbowAngle().getDegree() && requ_.getElbowAngle().getDegree() >= -100));

				System.out.println("requested state: " + requ_);

				System.out.println(((shouldMoveElevatorFirst) ? "We are moving the elevator first!" : "We are moving the arm first!"));

				return shouldMoveElevatorFirst;

			}
		};

		var choosePath = new ConditionalCommand(new SuperstructureGoToState(requ_), safeMove) {

			@Override
			protected boolean condition() {
				var proximalThreshold = -68;
				var currentState = SuperStructure.getInstance().getCurrentState();
				var nowOutsideCrossbar = currentState.getElbowAngle().getDegree() > proximalThreshold;
				var willBeOutsideCrossbar = requ_.getElbowAngle().getDegree() > proximalThreshold;

				var mightHitElectronics = (requ_.getElevatorHeight().getInch() < 15 && requ_.getElbowAngle().getDegree() > proximalThreshold) || (requ_.getElevatorHeight().getInch() < 20 && requ_.getElbowAngle().getDegree() < proximalThreshold); // TODO check angles?

				var proximalStartSafe = currentState.getElbowAngle().getDegree() > -80;
				var proximalEndSafe = requ_.getElbowAngle().getDegree() > -80;
				var startHighEnough = currentState.getElevatorHeight().getInch() > 18;
				var endHighEnough = requ_.getElevatorHeight().getInch() > 20;

				var needsExceptionForCargoGrab = currentState.getElbowAngle().getDegree() > -62 && currentState.getElevatorHeight().getInch() > 16;

				var safeToMoveSynced = (nowOutsideCrossbar && willBeOutsideCrossbar && (!mightHitElectronics || needsExceptionForCargoGrab))
						|| (proximalStartSafe && proximalEndSafe && startHighEnough && endHighEnough);

				SmartDashboard.putString("passthru data", "nowOutsideCrossbar " + nowOutsideCrossbar + " willBeOutsideCrossbar " + willBeOutsideCrossbar + " might hit electronics? " + mightHitElectronics +
						" proximalStartSafe " + proximalStartSafe + " proximalEndSafe? " + proximalEndSafe + " startHighEnough " + startHighEnough
						+ " endHighEnough " + endHighEnough);

				System.out.println("Safe to move synced? " + safeToMoveSynced);

				return safeToMoveSynced;
			}
		};

		var checkPassThroughSafe = new ConditionalCommand(new PassThroughButShort(SuperStructure.getInstance())) {

			@Override
			protected boolean condition() {
				var currentState = SuperStructure.getInstance().getCurrentState();
				var needsPassThrough = currentState.getElbowAngle().getDegree() < -90;
				System.out.println("Needs pass through? " + needsPassThrough);
				return needsPassThrough;
			}
		};

		addSequential(checkPassThroughSafe);
		addSequential(choosePath);

	}

	private class PassThroughButShort extends CommandGroup {
		public PassThroughButShort(SuperStructure structure) {
			addSequential(new PrintCommand("passing thru back to front"));
			addSequential(new ElevatorMove(LengthKt.getInch(23))); //todo check height
			addSequential(new PassThrough.SyncedMove(Math.toRadians(0), false, structure));
		}
	}

	public class ElevatorThanArm extends CommandGroup {
		public ElevatorThanArm(SuperStructureState requ) {
			addSequential(new ElevatorMove(requ.getElevator()));
			addSequential(new ArmMove(requ.getAngle()));
		}
	}

	public class ArmThanElevator extends CommandGroup {
		public ArmThanElevator(SuperStructureState requ) {
			addSequential(new ArmMove(requ.getAngle()));
			addSequential(new ElevatorMove(requ.getElevator()));
		}
	}

}
