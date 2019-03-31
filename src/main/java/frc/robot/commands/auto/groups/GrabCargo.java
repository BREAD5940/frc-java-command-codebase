package frc.robot.commands.auto.groups;

import org.team5940.pantry.experimental.command.SequentialCommandGroup;
import org.team5940.pantry.experimental.command.StartEndCommand;

import frc.robot.subsystems.Intake;

/** 
 * runs a series of commands to pick up a cargo from the ground
 */
public class GrabCargo extends SequentialCommandGroup {
	public GrabCargo() {
		super(
				new StartEndCommand(
						() -> {
							Intake.getInstance().setHatchSpeed(-1);
							Intake.getInstance().setCargoSpeed(1);
						},
						() -> {
							Intake.getInstance().setHatchSpeed(0);
							Intake.getInstance().setCargoSpeed(0);
						},
						Intake.getInstance()).withTimeout(1));
	}

}
