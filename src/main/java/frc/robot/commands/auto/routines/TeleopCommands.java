/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.drivetrain.HybridDriverAssist;
import frc.robot.commands.subsystems.superstructure.IntakeTelop;
import frc.robot.commands.subsystems.superstructure.JustElevatorTeleop;

public class TeleopCommands extends SendableCommandBaseGroup {
	/**
	 * Add your docs here.
	 */
	public TeleopCommands() {
		addParallel(new HybridDriverAssist());
		addParallel(new IntakeTelop());
		addParallel(new JustElevatorTeleop());
	}
}
