package frc.robot.commands.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.superstructure.Elevator;
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
    Length delta = LengthKt.getInch(Util.deadband(Robot.m_oi.getElevatorAxis() * 10 * Math.abs(Robot.m_oi.getElevatorAxis()), 0.08) );
		// Elevator elev = superStructure.getElevator(); // THIS LINE THROWS A HEKKING NULL PIONTER
		// FalconSRX<Length> talon = elev.getMaster();
    // Length current = talon.getSensorPosition();
    Length current = superStructure.getCurrentState().elevator.height;
		Length new_s = current.plus(delta);
		superStructure.moveSuperstructureElevator(new_s);
    // System.out.println("target height: " + new_s);
    
    // double power = Robot.m_oi.getElevatorAxis() * 1;
    // talon.set(ControlMode.PercentOutput, power);
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
