package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperStructureTelop extends Command {
	private OI mOI = Robot.m_oi;

	RoundRotation2d mLastWrist = SuperStructure.getInstance().getWrist().getRotation();
	RoundRotation2d mLastElbow = SuperStructure.getInstance().getElbow().getRotation();

	/** Run theSuperStructure.getInstance()(elevator for now) during telop using an xbox
	  * joystick. 
	  * @param struc theSuperStructure.getInstance()object
	  **/
	public SuperStructureTelop() {
		// Use requires() here to declare subsystem dependencies
		requires(SuperStructure.getInstance());
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		mLastWrist = SuperStructure.getInstance().getWrist().getRotation();
		mLastElbow = SuperStructure.getInstance().getElbow().getRotation();
	}

	boolean firstRunE = false;
	boolean firstRunL = false;
	boolean firstRunW = false;

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// elevator stuff
		Length newE = SuperStructure.getInstance().getLastReqElevatorHeight();
		RoundRotation2d newL = SuperStructure.getInstance().lastState.getElbowAngle();
		RoundRotation2d newW = SuperStructure.getInstance().lastState.getWrist().angle;

		boolean move = false;
		Length deltaE = LengthKt.getInch(Util.deadband(Robot.m_oi.getElevatorAxis() * 10 * Math.abs(Robot.m_oi.getElevatorAxis()), 0.08));
		if (Math.abs(Robot.m_oi.getElevatorAxis()) > 0.08) { // only move if asked
			firstRunE = true;
			Length currentE = SuperStructure.getInstance().getLastReqElevatorHeight();
			newE = currentE.plus(deltaE);
			move = true;

		} else {
			if (firstRunE == true) {
				newE = SuperStructure.getInstance().getLastReqElevatorHeight();
				firstRunE = false;
				move = true;
			}
		}

		if (Math.abs(Robot.m_oi.getWristAxis()) > 0.07) {
			firstRunL = true;
			newL = SuperStructure.getInstance().lastState.getElbow().angle.plus(
					RoundRotation2d.getDegree(Robot.m_oi.getWristAxis() * 10 * Math.abs(Robot.m_oi.getWristAxis()))); //FIXME check constant
			System.out.println("New elbow:" + newL.getDegree());
			move = true;
		} else {
			if (firstRunL) {
				newL = SuperStructure.getInstance().lastState.getElbowAngle();
				firstRunL = false;
				move = true;
			}
		}

		if (Math.abs(Robot.m_oi.getElbowAxis()) > 0.07) {
			firstRunW = true;
			move = true;
			newW = SuperStructure.getInstance().lastState.getWrist().angle.plus(
					RoundRotation2d.getDegree(Robot.m_oi.getElbowAxis() * 10 * Math.abs(Robot.m_oi.getElbowAxis()))); //FIXME check constant
		} else {
			if (firstRunW) {
				newW = SuperStructure.getInstance().lastState.getWrist().angle;
				firstRunW = false;
				move = true;
			}
		}

		if (move) {
			SuperStructure.getInstance().move(new SuperStructureState(new ElevatorState(newE), new RotatingArmState(newL), new RotatingArmState(newW)));
			SmartDashboard.putString("superstructure debugging", String.format("Current elevator req height: %f   Current elbow req angle: %f   Current wrist req angle: %f\n", newE.getInch(), newL.getDegree(), newW.getDegree()));
		}

		// move the whole darn thing
		// SuperStructure.getInstance().moveSuperstructureCombo(newReqE,SuperStructure.getInstance().getCurrentState().getElbow(), newReqW);
		// Logger.log("last wrist" + mLastWrist.getDegree() + " | current pos: " + SuperStructure.getInstance().getWrist().getDegrees());
		// Logger.log("last elbow" + mLastElbow.getDegree() + " | current pos: " + SuperStructure.getInstance().getElbow().getDegrees());

		// if (Math.abs(mOI.getWristAxis()) > 0.07) {
		// 	SuperStructure.getInstance().getWrist().getMaster().set(ControlMode.PercentOutput, Util.limit(Robot.m_oi.getWristAxis(), 0.75));
		// 	mLastWrist = SuperStructure.getInstance().getWrist().getPosition();
		// } else {
		// 	SuperStructure.getInstance().getWrist().getMaster().set(ControlMode.Position, mLastWrist);
		// }

		// if (Math.abs(mOI.getElbowAxis()) > 0.07) {
		// 	SuperStructure.getInstance().getElbow().getMaster().set(ControlMode.PercentOutput, Util.limit(Robot.m_oi.getElbowAxis() * 0.4, 0.75));
		// 	mLastElbow = SuperStructure.getInstance().getElbow().getPosition();
		// } else {
		// 	SuperStructure.getInstance().getElbow().getMaster().set(ControlMode.Position, mLastElbow);
		// }

		SmartDashboard.putNumber("Elbow current", SuperStructure.getInstance().getElbow().getMaster().getOutputCurrent());
		SmartDashboard.putNumber("Wrist current", SuperStructure.getInstance().getWrist().getMaster().getOutputCurrent());
		SmartDashboard.putNumber("Elevator current", SuperStructure.getInstance().getElevator().getMaster().getOutputCurrent());

		// double elbowDelta = Robot.m_oi.getElbowAxis();
		// Rotation2d newElbow = SuperStructure.getInstance().mReqState.getWrist().angle.plus(Rotation2dKt.getDegree(elbowDelta * 10));
		// SuperStructure.getInstance().getWrist().getMaster().set(ControlMode.Position, newElbow);

		// Logger.log("wrist percent output " + SuperStructure.getInstance().getWrist().getMaster().getMotorOutputPercent() + "wrist error raw: " + SuperStructure.getInstance().getWrist().getMaster().getClosedLoopError() + "new wrist setpoint: " + mLastWrist.getDegree());

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
