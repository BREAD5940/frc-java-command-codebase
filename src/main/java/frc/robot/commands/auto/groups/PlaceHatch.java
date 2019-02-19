package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.superstructure.RunIntake;

/**
 * runs a series of commands to place a hatch on the rocket or cargo ship
 */
public class PlaceHatch extends CommandGroup {
	public PlaceHatch() {
		// Move forward constantly until timed out while outtaking
		// addParallel(new DriveDistance(30, 10, 0.5)); // This should just keep going forward for like .5secs TODO test
		// Outtake hatch
		addSequential(new RunIntake(0.5, 2)); //FIXME this is only for testing. plz delet
		// addParallel(new CloseClamp()); //TODO make a CloseClamp command
	}

}
