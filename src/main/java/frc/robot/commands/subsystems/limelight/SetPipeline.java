/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.limelight;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.PipelinePreset;

public class SetPipeline extends Command {

	LimeLight ll = LimeLight.getInstance();
	int req;

	public SetPipeline(int req) {
		this.req = req;
	}

	public SetPipeline(PipelinePreset req_) {
		this(req_.getId());
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		ll.setPipeline(req);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
