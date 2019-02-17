package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.motion.Util;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperStructureTelop extends Command {
	private SuperStructure superStructure;

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

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// elevator stuff
		Length deltaE = LengthKt.getInch(Util.deadband(Robot.m_oi.getElevatorAxis() * 10 * Math.abs(Robot.m_oi.getElevatorAxis()), 0.08));
		Length currentE = superStructure.getCurrentState().elevator.height;
		Length newE = currentE.plus(deltaE);
		ElevatorState newReqE = new ElevatorState(newE);
		// superStructure.moveSuperstructureElevator(new_s);

		// jog wrist
		Rotation2d deltaW = Rotation2dKt.getDegree(Util.deadband(Robot.m_oi.getWristAxis() * 5 * Math.abs(Robot.m_oi.getWristAxis()), 0.08));
		Rotation2d currentW = superStructure.getCurrentState().getWrist().angle;
		Rotation2d newW = currentW.plus(deltaW);
		RotatingArmState newReqW = new RotatingArmState(newW);

		// move the whole darn thing
		// superStructure.moveSuperstructureCombo(newReqE, superStructure.getCurrentState().getElbow(), newReqW);
		superStructure.moveSuperstructureElevator(newE);

		superStructure.getWrist().getMaster().set(ControlMode.PercentOutput, Util.limit(Robot.m_oi.getWristAxis(), 0.5));
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
