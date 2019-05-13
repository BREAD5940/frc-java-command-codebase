/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines

import java.util.Arrays

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.*

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.robot.commands.auto.Trajectories
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode
import org.team5940.pantry.exparimental.command.SequentialCommandGroup

class Baseline : SequentialCommandGroup() {
    /**
     * Add your docs here.
     */
    init {

        val baseline = Trajectories.generateTrajectory(
                Arrays.asList(
                        Pose2d(5.188.feet,
                                17.652.feet,
                                0.degree),
                        Pose2d(9.94.feet,
                                17.564.feet,
                                0.degree),
                        Pose2d(17.156.feet,
                                24.206.feet,
                                30.degree)),
                Arrays.asList(CentripetalAccelerationConstraint(
                        8.0.feet.acceleration)),
                0.0.feet.velocity,
                0.0.feet.velocity,
                5.0.feet.velocity,
                8.0.feet.acceleration,
                false,
                true)

        addCommands(DriveTrain.getInstance().followTrajectory(baseline, TrajectoryTrackerMode.RAMSETE, true))

    }
}
