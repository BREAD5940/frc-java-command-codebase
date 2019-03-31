/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.team5940.pantry.experimental.command.InstantCommand;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.superstructure.SuperStructure;

/**
 * Add your docs here.
 */
public class KillAuto extends InstantCommand {
	/**
	 * Add your docs here.
	 */
	public KillAuto() {
		super();
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		addRequirements(SuperStructure.getInstance());
		addRequirements(SuperStructure.getInstance().getWrist());
		addRequirements(SuperStructure.getInstance().getElbow());
		addRequirements(SuperStructure.getElevator());
		addRequirements(DriveTrain.getInstance());
	}

	// Called once when the command executes
	@Override
	public void initialize() {
		SuperStructure.getInstance().getCurrentCommand().cancel();
		SuperStructure.getInstance().getWrist().getCurrentCommand().cancel();
		SuperStructure.getInstance().getElbow().getCurrentCommand().cancel();
		SuperStructure.getElevator().getCurrentCommand().cancel();
		DriveTrain.getInstance().getCurrentCommand().cancel();
		DriveTrain.getInstance().stop();
		var currentState = SuperStructure.getInstance().getCurrentState();
		SuperStructure.getInstance().move(currentState);
	}

}
