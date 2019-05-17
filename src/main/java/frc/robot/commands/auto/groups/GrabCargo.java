package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.superstructure.RunIntake;

/** 
 * runs a series of commands to pick up a cargo from the ground
 */
public class GrabCargo extends SendableCommandBaseGroup {
	public GrabCargo() {
		addSequential(new RunIntake(1, 1, 1)); // FIXME check runtime
	}

}
