/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.Robot;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.Intake;

public class IntakeTelop extends SendableCommandBase {
	public IntakeTelop() {
		// Use addRequirements() here to declare subsystem dependencies
		addRequirements(Intake.getInstance());
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		if (Math.abs(Robot.m_oi.getCargoSpeed()) > 0.2) {
			Intake.getInstance().setSpeed(-1 * Robot.m_oi.getCargoSpeed(),
					Robot.m_oi.getCargoSpeed());
		} else {
			Intake.getInstance().setSpeed(Robot.m_oi.getHatchSpeed(), 0);
		}
		var oi = Robot.m_oi;
		oi.setAllRumble(Util.limit(
				Math.max(Robot.m_oi.getCargoSpeed(), Robot.m_oi.getHatchSpeed()), 0,
				0.9));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {}

}
