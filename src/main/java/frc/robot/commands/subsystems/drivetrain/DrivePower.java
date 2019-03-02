/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.subsystems.DriveTrain;

public class DrivePower extends Command {
	double power, time, reqEndTime;

	/**
	 * So I fear no man. But this, this scares me. Literally drive forward for a couple seconds.
	 * @param power
	 * @param time
	 */
	public DrivePower(double power, double time) {
		// super(arg0, arg1)
		// Use requires() here to declare subsystem dependencies
		requires(DriveTrain.getInstance());
		this.power = power;
		this.time = time;
		// setTimeout(time);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		reqEndTime = time + Timer.getFPGATimestamp();
		System.out.println("Requested end time: " + reqEndTime + "now: " + Timer.getFPGATimestamp());
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// System.out.println("hi!");
		DriveTrain.getInstance().arcadeDrive(power, 0, false);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		System.out.println("Now: " + Timer.getFPGATimestamp() + "end: " + reqEndTime + " is drive power timed out? " + (Timer.getFPGATimestamp() > reqEndTime));
		return (Timer.getFPGATimestamp() > reqEndTime);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		DriveTrain.getInstance().stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
