/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.obj.factories;

import java.util.List;

import org.team5940.pantry.exparimental.command.Command;
import org.team5940.pantry.exparimental.command.SendableCommandBase;
import org.team5940.pantry.exparimental.command.SequentialCommandGroup;

/**
 * Maybe a factory for making preset command groups. This should be used for things like
 * level one hatch, where you have to move the wrist before the elevator. Or for lazy stuff
 * 
 * @author Matthew Morley
 */
public class SequentialCommandFactory {

	public static SendableCommandBase getSequentialCommands(List<Command> toAdd) {
		var toReturn = new SequentialCommandGroup();
		for (Command it : toAdd) {
			toReturn.addCommands(it);
		}
		return toReturn;
	}

}
