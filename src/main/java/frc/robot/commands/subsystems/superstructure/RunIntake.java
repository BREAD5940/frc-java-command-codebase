/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.exparimental.command.WaitCommand;

import frc.robot.subsystems.Intake;

/**
 * Add your docs here.
 */
public class RunIntake extends WaitCommand {
	/**
	 * Run the intake at some speeds for a time
	 */
	double hatch, cargo;

	public RunIntake(double hatchSpeed, double cargoSpeed, double timeout) {
		super(timeout);
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		addRequirements(Intake.getInstance());
		this.hatch = hatchSpeed;
		this.cargo = cargoSpeed;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		Intake.getInstance().setCargoSpeed(cargo);
		Intake.getInstance().setHatchSpeed(hatch);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		Intake.getInstance().setCargoSpeed(cargo);
		Intake.getInstance().setHatchSpeed(hatch);

	}

	// Called once after timeout
	protected void end() {
		Intake.getInstance().setCargoSpeed(0);
		Intake.getInstance().setHatchSpeed(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			interrupted();
		} else
			end();
	}
}
