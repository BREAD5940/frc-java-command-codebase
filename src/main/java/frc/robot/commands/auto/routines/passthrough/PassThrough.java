/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines.passthrough;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.subsystems.superstructure.SuperStructure;

public class PassThrough extends CommandGroup {
	/**
	 * Add your docs here.
	 */
	public PassThrough() {
		// addSequential( new ConditionalCommand(new PassThroughForward(), new PassThroughReverse()){
		//   @Override
		//   protected boolean condition() {
		//     SuperStructure.getInstance().updateState();
		//     return (SuperStructure.getInstance().getCurrentState().jointAngles.elbowAngle.angle.getDegree() > -90 );
		//   }
		// });

		Command decider = new ConditionalCommand(new PassThroughForward(), new PassThroughReverse()) {
			@Override
			protected boolean condition() {
				SuperStructure.getInstance().updateState();
				double degrees = SuperStructure.getInstance().getCurrentState().jointAngles.elbowAngle.angle.getDegree();
				boolean direction = (degrees > -90);
				return (SuperStructure.getInstance().getCurrentState().jointAngles.elbowAngle.angle.getDegree() > -90);
			}
		};

		addSequential(decider);

	}
}
