package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * basically just a CommandGroup but with the done() method
 */
public class AutoCommandGroup extends CommandGroup {

	public AutoCommandGroup() {}

	public boolean done() {
		return this.isFinished();
	}

}
