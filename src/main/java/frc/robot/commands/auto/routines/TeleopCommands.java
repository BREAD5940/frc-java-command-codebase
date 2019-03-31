/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import org.team5940.pantry.experimental.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.drivetrain.HybridDriverAssist;
import frc.robot.commands.subsystems.superstructure.IntakeTelop;
import frc.robot.commands.subsystems.superstructure.JustElevatorTeleop;

public class TeleopCommands extends ParallelCommandGroup {
	/**
	 * Add your docs here.
	 */
	public TeleopCommands() {

		addCommands(
			new HybridDriverAssist(),
			new IntakeTelop(),
			new JustElevatorTeleop()
		);
	}
}
