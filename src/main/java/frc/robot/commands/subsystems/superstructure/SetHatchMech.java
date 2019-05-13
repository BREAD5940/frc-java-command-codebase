/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.exparimental.command.InstantCommand;

import frc.robot.lib.Logger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.HatchMechState;

public class SetHatchMech extends InstantCommand {
	public SetHatchMech(HatchMechState mReq_) {
		super(() -> Intake.getInstance().setHatchMech(mReq_));
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		Logger.log("setting hatch mech to " + mReq_.name());
	}
}
