/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.obj.factories;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * Add your docs here.
 */
public class SequentialCommandFactory {

	public static CommandGroup levelOneHatch() {
		CommandGroup toReturn = new CommandGroup();
		toReturn.addSequential(new SuperstructureGoToState(iPosition.HATCH), 3);
		toReturn.addSequential(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal)), 3);

		return toReturn;

	}

	// public static Command getSequentialCommands(Command... toAdd) {
	//   CommandGroup toReturn = new CommandGroup();
	//   for(Command c : toAdd) {
	//     toReturn.addSequential(c);
	//   }
	//   return toReturn;
	// }

}
