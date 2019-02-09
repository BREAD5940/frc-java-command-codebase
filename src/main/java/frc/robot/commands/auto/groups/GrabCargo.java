package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.actions.SetIntakeMode;
import frc.robot.commands.subsystems.superstructure.RunIntake;

/** 
 * runs a series of commands to pick up a cargo from the ground
 */
public class GrabCargo extends CommandGroup {
	public GrabCargo() {
		// addSequential(new SetElevatorHeight(0, true));
		addSequential(new SetIntakeMode(HeldPiece.CARGO, true));
		addSequential(new RunIntake(50, 1)); // FIXME check runtime
	}

}
