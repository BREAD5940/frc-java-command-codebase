package frc.robot.commands.auto.groups;

import org.team5940.pantry.exparimental.command.SequentialCommandGroup;

import frc.robot.commands.subsystems.superstructure.RunIntake;

/** 
 * runs a series of commands to pick up a cargo from the ground
 */
public class GrabCargo extends SequentialCommandGroup {
	public GrabCargo() {
		addCommands(new RunIntake(1, 1, 1)); // FIXME check runtime
	}

}
