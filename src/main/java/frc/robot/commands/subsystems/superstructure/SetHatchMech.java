/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.experimental.command.InstantCommand;

import frc.robot.lib.Logger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.HatchMechState;

public class SetHatchMech extends InstantCommand {

	private HatchMechState mReq;

	public SetHatchMech(HatchMechState mReq_) {
		// super("Set hatch mech");
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		this.mReq = mReq_;
		Logger.log("setting hatch mech to " + mReq_.name());
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		Intake.getInstance().setHatchMech(mReq);
	}

}
