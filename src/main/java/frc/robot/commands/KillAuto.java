/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
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
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(SuperStructure.getInstance());
		requires(SuperStructure.getInstance().getWrist());
		requires(SuperStructure.getInstance().getElbow());
		requires(SuperStructure.getElevator());
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
    SuperStructure.getInstance().getCurrentCommand().cancel();
		SuperStructure.getInstance().getWrist().getCurrentCommand().cancel();
		SuperStructure.getInstance().getElbow().getCurrentCommand().cancel();
		SuperStructure.getElevator().getCurrentCommand().cancel();
  }

}
