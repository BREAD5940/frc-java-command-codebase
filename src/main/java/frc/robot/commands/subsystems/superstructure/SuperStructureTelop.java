package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperStructureTelop extends Command {
	private OI mOI = Robot.m_oi;

	SuperStructureState mCachedState;

	/** Run theSuperStructure.getInstance()(elevator for now) during telop using an xbox
	  * joystick. 
	  * @param struc theSuperStructure.getInstance()object
	  **/
	public SuperStructureTelop() {
		// Use requires() here to declare subsystem dependencies
		requires(SuperStructure.getInstance());
		requires(SuperStructure.getInstance().getWrist());
		requires(SuperStructure.getInstance().getElbow());
		requires(SuperStructure.getElevator());
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		SuperStructure.elevator.setGear(Elevator.kDefaultGear);
		mCachedState = SuperStructure.getInstance().updateState();
	}

	boolean firstRun = false;

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		SuperStructureState mCurrentState = SuperStructure.getInstance().updateState();
		var mNewState = new SuperStructureState();
		var mElevatorPower = Robot.m_oi.getElevatorAxis();
		var mWristPower = Robot.m_oi.getWristAxis();
		var mElbowPower = Robot.m_oi.getElbowAxis();
		var kDeadband = 0.05d;

		// Figure out of the operator is commanding an elevator move. If so, increment the new state and cache the current state - if not, stay at the cached state.
		if (Math.abs(mElevatorPower) > kDeadband) {
			mNewState.elevator = new ElevatorState(mCurrentState.getElevatorHeight().plus(LengthKt.getInch(mElevatorPower * ((mElevatorPower > 0) ? 5 : 10))));
			mCachedState.elevator = mNewState.elevator;
		} else {
			mNewState.elevator = mCachedState.elevator;
		}

		// Figure out of the operator is commanding a wrist move. If so, increment the new state and cache the current state - if not, stay at the cached state.
		if (Math.abs(mWristPower) > kDeadband) {
			mNewState.jointAngles.wristAngle = new RotatingArmState(mCurrentState.getWrist().angle.plus(RoundRotation2d.getDegree(mWristPower * 15)));
			// System.out.println("new wrist angle: " + mNewState.jointAngles.wristAngle.angle.getDegree() + "old wrist angle: " + mCurrentState.getWrist().angle.getDegree());
			mCachedState.jointAngles.wristAngle = mNewState.jointAngles.wristAngle;
		} else {
			mNewState.jointAngles.wristAngle = mCachedState.jointAngles.wristAngle;
		}

		// Figure out of the operator is commanding a elbow move. If so, increment the new state and cache the current state - if not, stay at the cached state.
		if (Math.abs(mElbowPower) > kDeadband) {
			mNewState.jointAngles.elbowAngle = new RotatingArmState(mCurrentState.getElbow().angle.plus(RoundRotation2d.getDegree(mElbowPower * 5)));
			mCachedState.jointAngles.elbowAngle = mNewState.jointAngles.elbowAngle;
		} else {
			mNewState.jointAngles.elbowAngle = mCachedState.jointAngles.elbowAngle;
		}

		SuperStructure.getInstance().move(mNewState);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
