/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.team5940.pantry.experimental.command.SequentialCommandGroup;

import frc.robot.commands.auto.Trajectories;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

public class Baseline extends SequentialCommandGroup {
	/**
	 * Add your docs here.
	 */
	public Baseline() {

		var baseline = Trajectories.generateTrajectory(
				Arrays.asList(
						new Pose2d(LengthKt.getFeet(5.188),
								LengthKt.getFeet(17.652),
								Rotation2dKt.getDegree(0)),
						new Pose2d(LengthKt.getFeet(9.94),
								LengthKt.getFeet(17.564),
								Rotation2dKt.getDegree(0)),
						new Pose2d(LengthKt.getFeet(17.156),
								LengthKt.getFeet(24.206),
								Rotation2dKt.getDegree(30))),
				Arrays.asList(new CentripetalAccelerationConstraint(
						AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)))),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(5.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		addCommands(DriveTrain.getInstance().followTrajectory(baseline, TrajectoryTrackerMode.RAMSETE, true));

	}
}
