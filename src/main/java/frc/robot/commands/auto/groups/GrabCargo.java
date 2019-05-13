package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import org.team5940.pantry.exparimental.command.SequentialCommandGroup;

/** 
 * runs a series of commands to pick up a cargo from the ground
 */
public class GrabCargo extends SequentialCommandGroup {
	public GrabCargo() {
		addCommands(new RunIntake(1, 1, 1)); // FIXME check runtime
	}

}
