/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines.passthrough;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PassThroughReverse extends AutoCommandGroup {
	/**
	 * Pass through the elevator from front to back
	 */
	public PassThroughReverse() {

		SuperStructureState passthroughInitState = SuperstructurePlanner.passThroughState;

		SuperStructureState passedThrough = new SuperStructureState(
				new ElevatorState(LengthKt.getInch(26)),
				new RotatingArmState(RoundRotation2d.getDegree(-180)),
				new RotatingArmState(RoundRotation2d.getDegree(-120)));

		addSequential(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal), iPosition.HATCH_REVERSE));
		addSequential(new SuperstructureGoToState(passedThrough));
		addSequential(new SuperstructureGoToState(passthroughInitState));
		addSequential(new SuperstructureGoToState(passthroughInitState.elevator, iPosition.HATCH));
		addSequential(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal), iPosition.HATCH));
	}

}
