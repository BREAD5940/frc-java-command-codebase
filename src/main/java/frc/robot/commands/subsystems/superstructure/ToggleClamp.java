/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.HatchMechState;

/**
 * Add your docs here.
 */
public class ToggleClamp extends InstantCommand {
	/**
	 * Add your docs here.
	 */
	public ToggleClamp() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
		var currentState = Intake.getInstance().getHatchMechState();
		var newState = (currentState == HatchMechState.kClamped) ? HatchMechState.kOpen : HatchMechState.kClamped;
		System.out.println("OLD STATE: " + currentState.name() + " NEW STATE: " + newState.name());
		Intake.getInstance().setHatchMech(newState);
	}

}
