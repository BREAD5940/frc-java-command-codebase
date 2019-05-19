/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import org.team5940.pantry.exparimental.command.InstantCommand;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.Gear;
//import org.team5940.pantry.exparimental.command.InstantCommand;
//import org.team5940.pantry.exparimental.command.InstantCommand;

/**
 * Add your docs here.
 */
public class SetGearCommand extends InstantCommand {
	Gear gear;

	/**
	 * Add your docs here.
	 */
	public SetGearCommand(Gear gear) {
		//		super("Set gear to " + gear.name());
		this.gear = gear;
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
	}

	// Called once when the command executes
	@Override
	public void initialize() {
		Robot.drivetrain.setGear(gear);
	}

}
