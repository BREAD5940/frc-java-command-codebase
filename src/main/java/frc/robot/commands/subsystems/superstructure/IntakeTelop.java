/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.superstructure.SuperStructure;

public class IntakeTelop extends Command {
	public IntakeTelop() {
		// Use requires() here to declare subsystem dependencies
		requires(SuperStructure.intake);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (Math.abs(Robot.m_oi.getCargoSpeed()) > 0.2) {
			SuperStructure.intake.setSpeed(-1 * Robot.m_oi.getCargoSpeed(), Robot.m_oi.getCargoSpeed());
		} else {
			SuperStructure.intake.setSpeed(Robot.m_oi.getHatchSpeed(), 0);
		}
		var oi = Robot.m_oi;
		oi.setAllRumble(Util.limit(Math.max(Robot.m_oi.getCargoSpeed(), Robot.m_oi.getHatchSpeed()), 0, 0.9));
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
