/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import org.team5940.pantry.experimental.command.SendableCommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class DrivePowerAndIntake extends SendableCommandBase {
	double power, reqEndTime, intake;

	/**
	 * So I fear no man. But this, this scares me. Literally drive forward for a couple seconds.
	 * @param power
	 * @param time
	 * 
	 * @author Matthew Morley
	 */
	public DrivePowerAndIntake(double drive, double intake, double time) {
		// super(time);
		// Use addRequirements() here to declare subsystem dependencies
		addRequirements(DriveTrain.getInstance());
		addRequirements(Intake.getInstance());
		this.power = drive;
		this.intake = intake;
		// this.time = time;
		// setTimeout(time);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// System.out.println("hi!");
		DriveTrain.getInstance().arcadeDrive(power, 0, false);
		Intake.getInstance().setHatchSpeed(intake);
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		DriveTrain.getInstance().stop();
		Intake.getInstance().setHatchSpeed(0);
	}

}
