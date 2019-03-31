package frc.robot.commands.auto.groups;

import org.team5940.pantry.experimental.command.SequentialCommandGroup;
import org.team5940.pantry.experimental.command.StartEndCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.subsystems.Intake;

public class DropCargo extends SequentialCommandGroup {
	/**
	 * drops cargo. 
	 * if it's being shot through a hatch/port, it outtakes forward. otherwise it outtakes downward
	 * @param isDrop
	 *    if the cargo is being dropped into the cargo ship or not
	 */
	public DropCargo() {

		addCommands(
			new StartEndCommand(
				() -> {Intake.getInstance().setHatchSpeed(1);
						Intake.getInstance().setCargoSpeed(-1);		
				},
				() -> {Intake.getInstance().setHatchSpeed(0);
					Intake.getInstance().setCargoSpeed(0);},
				Intake.getInstance()		
			).withTimeout(1));

	}

}
