package frc.robot.commands.auto.groups;

import org.team5940.pantry.exparimental.command.SequentialCommandGroup;

import frc.robot.commands.subsystems.superstructure.RunIntake;

public class DropCargo extends SequentialCommandGroup {
	/**
	 * drops cargo. 
	 * if it's being shot through a hatch/port, it outtakes forward. otherwise it outtakes downward
	 * @param isDrop
	 *    if the cargo is being dropped into the cargo ship or not
	 */
	public DropCargo(boolean isDrop) {
		if (!isDrop) {
			// addSequential(new SetWrist(0, false));
			addCommands(new RunIntake(-1, -1, 2));
		} else {
			// addSequential(new SetWrist(-90, false)); // TODO the entire SetWrist command is currently commented out
			addCommands(new RunIntake(-1, 01, 2));
		}

	}

}
