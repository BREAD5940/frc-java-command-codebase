package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.lib.Logger;
import frc.robot.lib.motion.Util;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperStructureTelop extends Command {
	private SuperStructure superStructure;
	private OI mOI = Robot.m_oi;

	/** Run the superstructure (elevator for now) during telop using an xbox
	  * joystick. 
	  * @param struc the superstructure object
	  **/
	public SuperStructureTelop(SuperStructure struc) {
		// Use requires() here to declare subsystem dependencies
		requires(struc);
		this.superStructure = struc;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {}

	boolean firstRun = false;
	Rotation2d mLastWrist = Rotation2dKt.getDegree(0), mLastElbow = Rotation2dKt.getDegree(0);

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// elevator stuff
		Length newE = superStructure.getLastReqElevatorHeight();
		boolean move = false;
		Length deltaE = LengthKt.getInch(Util.deadband(Robot.m_oi.getElevatorAxis() * 10 * Math.abs(Robot.m_oi.getElevatorAxis()), 0.08));
		if (Math.abs(Robot.m_oi.getElevatorAxis()) > 0.08) { // only move if asked
			firstRun = true;
			Length currentE = superStructure.getLastReqElevatorHeight();
			newE = currentE.plus(deltaE);
			ElevatorState newReqE = new ElevatorState(newE);
			move = true;
			// superStructure.moveSuperstructureElevator(new_s);

		} else {
			if (firstRun == true) {
				newE = superStructure.getLastReqElevatorHeight();
				firstRun = false;
				move = true;
			}
		}

		if (move) {
			superStructure.moveSuperstructureElevator(newE);
		}

		// move the whole darn thing
		// superStructure.moveSuperstructureCombo(newReqE, superStructure .getCurrentState().getElbow(), newReqW);

		if (Math.abs(mOI.getWristAxis()) > 0.07) {
			superStructure.getWrist().getMaster().set(ControlMode.PercentOutput, Util.limit(Robot.m_oi.getWristAxis(), 0.75));
			mLastWrist = superStructure.getWrist().getPosition();
		} else {
			superStructure.getWrist().getMaster().set(ControlMode.Position, mLastWrist);
		}

		if (Math.abs(mOI.getElbowAxis()) > 0.07) {
			superStructure.getElbow().getMaster().set(ControlMode.PercentOutput, Util.limit(Robot.m_oi.getElbowAxis(), 0.75));
			mLastElbow = superStructure.getElbow().getPosition();
		} else {
			superStructure.getElbow().getMaster().set(ControlMode.Position, mLastElbow);
		}

		// double elbowDelta = Robot.m_oi.getElbowAxis();
		// Rotation2d newElbow = superStructure.mReqState.getWrist().angle.plus(Rotation2dKt.getDegree(elbowDelta * 10));
		// superStructure.getWrist().getMaster().set(ControlMode.Position, newElbow);

		// Logger.log("wrist percent output " + superStructure.getWrist().getMaster().getMotorOutputPercent() + "wrist error raw: " + superStructure.getWrist().getMaster().getClosedLoopError() + "new wrist setpoint: " + mLastWrist.getDegree());

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
