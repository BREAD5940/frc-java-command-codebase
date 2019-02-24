/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
	double demand, duratuion;

	/**
	 * Run the intake for a set amount of time
	 * @param demand
	 * @param duration
	 */
	public RunIntake(double demand_, double duration_) {
		requires(Intake.getInstance());
		this.demand = demand_;
		this.duratuion = duration_;
		setTimeout(duration_);
	}

	@Override
	protected void initialize() {
		Intake.getInstance().setSpeed(demand);
	}

	@Override
	protected void execute() {}

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	@Override
	protected void end() {
		Intake.getInstance().setSpeed(0d);
	}

	@Override
	protected void interrupted() {}
}
