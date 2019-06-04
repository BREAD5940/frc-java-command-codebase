package frc.robot.commands.auto.routines.offseasonRoutines

import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.command.WaitCommand
import frc.robot.RobotConfig
import frc.robot.commands.auto.Autonomous
import frc.robot.commands.auto.paths.TrajectoryFactory
import frc.robot.commands.auto.paths.TrajectoryWaypoints
import frc.robot.commands.auto.routines.AutoRoutine
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.commands.subsystems.superstructure.PassThrough
import frc.robot.commands.subsystems.superstructure.RunIntake
import frc.robot.commands.subsystems.superstructure.SetHatchMech
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals


class BottomRocketRoutine(isLeft: Boolean) : AutoRoutine() {

    // First path goes to the far side of the rocket
    private val path1 = TrajectoryFactory.sideStartToRocketF

    // Second path goes to the loading station to pick up a hatch panel
    private val path2 = TrajectoryFactory.rocketFToLoadingStation

    // Third path goes to the near side of the rocket
    private val path3 = TrajectoryFactory.loadingStationToRocketN


    // Calculates the duration of the path
    val duration = path1.duration + path2.duration + path3.duration


    // Auto routine
init {

        +InstantCommand{
            DriveTrain.getInstance().localization.reset(
                    if(isLeft) TrajectoryWaypoints.kSideStart.mirror else TrajectoryWaypoints.kSideStart
            )
        }

        +SetHatchMech(Intake.HatchMechState.kClamped)


        // Part 1: Go to the far side of the rocket and get ready to place a hatch on the lowest level.
        +parallel(
                // Follow the trajectory with vision correction to the far side of the rocket.
                super.followVisionAssistedTrajectory(
                    path1,
                        {isLeft},
                    4.feet, true
                ),
                // Take the superstructure to scoring height once out of the platform.
                sequential(
                        WaitCommand(1.second.second),
//                    notWithinRegion(TrajectoryWaypoints.kHabitatL1Platform),
//                    +Superstructure.kFrontHatchFromLoadingStation
                        JankyGoToState(RobotConfig.auto.fieldPositions.hatchLowGoal, SuperStructure.iPosition.HATCH))

        )


            // Reorient for rocketF
            +relocalize(
                    TrajectoryWaypoints.kRocketF,
                true
            ) {isLeft}

        val path2 = super.followVisionAssistedTrajectory(
                path2,
                {isLeft},
                4.feet, false
            )

            // Part 2: Place hatch and go to loading station.
            +parallel(
                // Follow the trajectory with vision correction to the loading station.
                path2,
                // Take the superstructure to pickup position and arm hatch intake 3 seconds before arrival.
                sequential(
                        // release hatch panel
                        RunIntake(1.0, 0.0, 0.5),
                        Intake.getInstance().clamp(),
                        PassThrough.FrontToBack(SuperStructure.getInstance()),
                        RunIntake(-1.0, 0.0, 0.5).withExit { path2.isCompleted }
                )
            )

            // Reorient position on field based on Vision alignment.
            +relocalize(
                    TrajectoryWaypoints.kLoadingStation,
                false,
                {isLeft}
            )

            // Part 3: Pickup hatch and go to the near side of the rocket.
            +parallel(
                    // Make sure the intake is holding the hatch panel.
                    RunIntake(-1.0, 0.0, 0.5),
                    // Follow the trajectory with vision correction to the near side of the rocket.
                    super.followVisionAssistedTrajectory(
                            path3,
                            {isLeft},
                            6.feet, true
                    ),
                    // Take the superstructure to scoring height.
                    sequential(
                            WaitCommand(1.second.second),
                            JankyGoToState(RobotConfig.auto.fieldPositions.hatchLowGoal, SuperStructure.iPosition.HATCH)
                    )
            )

            // Part 4: Score the hatch and go to the loading station for the end of the sandstorm period.
            +parallel(
                // Score hatch.
                // Follow the trajectory to the loading station.
                DriveTrain.getInstance().followTrajectory(
                        TrajectoryFactory.rocketNToLoadingStation,
                        false,
                        {isLeft}
                ),
                // Take the superstructure to a position to pick up the next hatch.
                sequential(
                        RunIntake(-1.0, 0.0, 0.5),
                        Intake.getInstance().clamp(),
                        PassThrough.FrontToBack(SuperStructure.getInstance())
                )
            )
    }
}