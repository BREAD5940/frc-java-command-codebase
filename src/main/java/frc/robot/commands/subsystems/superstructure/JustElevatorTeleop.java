package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.SuperStructure;

public class JustElevatorTeleop extends SendableCommandBase {
	private OI mOI = Robot.m_oi;

	SuperStructureState mCachedState;

	private static Length mOffset = LengthKt.getInch(0);

	/**
	 * Jog the superstructure using an xbox controller. Mainly used for testing.
	 * 
	 * @author Matthew Morley
	 */
	public JustElevatorTeleop() {
		// Use addRequirements() here to declare subsystem dependencies
		addRequirements(SuperStructure.getInstance());
		// addRequirements(SuperStructure.getInstance().getWrist());
		// addRequirements(SuperStructure.getInstance().getElbow());
		addRequirements(SuperStructure.getElevator());
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		SuperStructure.elevator.setGear(Elevator.kDefaultGear);
		// System.out.println("kp: ================ " + SuperStructure.elevator.getMaster().getKP());
		mCachedState = SuperStructure.getInstance().updateState();
		SuperStructure.getElevator().getMaster().getMotorController().selectProfileSlot(3, 0);
		// SuperStructure.getInstance().getElbow().getMaster().selectProfileSlot(3, 0);

		if (SuperStructure.elevator.elevatorZeroed == false) {
			System.out.println("ELEVATOR NOT ZEROED REEEEEEEEEEEEEEEEE");
			this.cancel();
		}
	}

	boolean firstRun = false;

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {

		SuperStructureState mCurrentState = SuperStructure.getInstance().updateState();
		var mNewState = mCachedState;
		var mElevatorPower = Robot.m_oi.getElevatorDS();
		// var mWristPower = Robot.m_oi.getWristAxis();
		// var mElbowPower = Robot.m_oi.getElbowAxis();
		// mElbowPower = mElbowPower * Math.abs(mElbowPower); // square inputs
		var kDeadband = 0.15d;

		// Figure out of the operator is commanding an elevator move. If so, increment the new state and cache the current state - if not, stay at the cached state.
		if (Math.abs(mElevatorPower) > 0.01) {
			mNewState.elevator = new ElevatorState(mCurrentState.getElevatorHeight().plus(LengthKt.getInch(mElevatorPower * ((mElevatorPower > 0) ? 2 : 2))));
			mCachedState.elevator = mNewState.elevator;
		} //else {
			// mNewState.elevator = mCachedState.elevator;
			// }

		// Figure out of the operator is commandi
		SuperStructure.getInstance().move(mNewState);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		var currentState = SuperStructure.getInstance().getCurrentState();
		SuperStructure.elevator.setPositionSetpoint(currentState);
	}
}
