/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;

import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.commands.auto.actions.DelayCommand;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

public class PassThrough extends AutoCommandGroup {
	/**
	 * Add your docs here.
	 */
	public PassThrough() {
		// 25.987168527570645,-99.67531651929261,-58.062744140625,NONE

		SuperStructureState passthroughInitState = SuperstructurePlanner.passThroughState;
		// 24.692488742595167,-179.08774618167203,-117.97119140625,NONE

		SuperStructureState passedThrough = new SuperStructureState(
				new ElevatorState(LengthKt.getInch(26)),
				new RotatingArmState(RoundRotation2d.getDegree(-180)),
				new RotatingArmState(RoundRotation2d.getDegree(-120)));

		addSequential(new SuperstructureGoToState(passthroughInitState));
		addSequential(new PrintCommand("At the bottom - waiting..."));
		addSequential(new DelayCommand(TimeUnitsKt.getSecond(1)));
		// addSequential(new SuperstructureGoToState(straightDown));
		// addSequential(new DelayCommand(TimeUnitsKt.getSecond(2)));
		addSequential(new PrintCommand("Passing Though..."));
		addSequential(new SuperstructureGoToState(passedThrough));

		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
	}
}
