/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.obj.factories;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * Maybe a factory for making preset command groups. This should be used for things like
 * level one hatch, where you have to move the wrist before the elevator. Or for lazy stuff
 * 
 * @author Matthew Morley
 */
public class SequentialCommandFactory {

	public static CommandGroup levelOneHatch() {
		CommandGroup toReturn = new CommandGroup();
		toReturn.addSequential(new SuperstructureGoToState(iPosition.HATCH));
		toReturn.addSequential(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal)));

		return toReturn;

	}

	public static CommandGroup getSequentialCommands(List<Command> toAdd) {
		CommandGroup toReturn = new CommandGroup();
		for (Command c : toAdd) {
			toReturn.addSequential(c);
		}
		return toReturn;
	}

}
