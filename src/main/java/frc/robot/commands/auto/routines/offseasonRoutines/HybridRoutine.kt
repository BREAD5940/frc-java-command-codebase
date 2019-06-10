package frc.robot.commands.auto.routines.offseasonRoutines

import edu.wpi.first.wpilibj.command.WaitCommand
import frc.robot.RobotConfig
import frc.robot.commands.auto.paths.TrajectoryFactory
import frc.robot.commands.auto.paths.TrajectoryWaypoints
import frc.robot.commands.auto.routines.AutoRoutine
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.commands.subsystems.superstructure.PassThrough
import frc.robot.commands.subsystems.superstructure.RunIntake
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.Source

class HybridRoutine(private val mode: Mode) : AutoRoutine() {

    enum class Mode(
        val path1: TimedTrajectory<Pose2dWithCurvature>,
        val path2: TimedTrajectory<Pose2dWithCurvature>,
        val path3: TimedTrajectory<Pose2dWithCurvature>,
        val isLeft: Boolean
    ) {
        LEFT(
                TrajectoryFactory.centerStartToCargoShipFL,
                TrajectoryFactory.cargoShipFLToLeftLoadingStation,
            TrajectoryFactory.loadingStationToRocketN.mirror(),
            true
        ),
        RIGHT(
                TrajectoryFactory.centerStartToCargoShipFR,
                TrajectoryFactory.cargoShipFRToRightLoadingStation,
                TrajectoryFactory.loadingStationToRocketN,
            false
        )
    }

    val duration: Time
        get() = mode.path1.duration + mode.path2.duration + mode.path3.duration

    init {
            +parallel(
                    // drive to cargo ship FL
                followVisionAssistedTrajectory(mode.path1, { false }, 4.feet, true),
                sequential(
                        WaitCommand(Math.min((mode.path1.duration - 3.5.second).second, 0.01)),
                        JankyGoToState(RobotConfig.auto.fieldPositions.hatchLowGoal, SuperStructure.iPosition.HATCH)
                )
            )

            val path2 = followVisionAssistedTrajectory(mode.path2, { false }, 4.feet)

            +parallel(
                path2, // drive from caro ship to left loading station
                sequential(
                        RunIntake.exhaustHatchFor(0.5.second),
                    Intake.getInstance().clamp(),
                    PassThrough.FrontToBack(SuperStructure.getInstance()),
                        AutoRoutine(RunIntake.grabHatchFor(mode.path2.duration))
                                .withExit { path2.isCompleted }
                )
            )

            +relocalize(TrajectoryWaypoints.kLoadingStation, false, Source(mode.isLeft))

            +parallel(
                // Make sure the intake is holding the hatch panel.
                RunIntake.grabHatchFor(0.5.second),
                // Follow the trajectory with vision correction to the near side of the rocket.
                super.followVisionAssistedTrajectory(
                    mode.path3, // loadingStationToRocketN
                    { false },
                    4.feet, true
                ),
                // Take the superstructure to scoring height.
                    sequential(
                            WaitCommand(0.3.second.second),
                            JankyGoToState(RobotConfig.auto.fieldPositions.hatchLowGoal, SuperStructure.iPosition.HATCH)
                    )
            )

            // Part 4: Score the hatch and go to the loading station for the end of the sandstorm period.
            +parallel(
                    // Follow the trajectory to the loading station.
                DriveTrain.getInstance().followTrajectory(
                        TrajectoryFactory.rocketNToLoadingStation,
                        false,
                        Source(mode.isLeft)
                ),
                sequential(
                // Score hatch.
                        RunIntake.grabHatchFor(0.5.second),
                    Intake.getInstance().clamp(),
                // Take the superstructure to a position to pick up the next hatch.
                    PassThrough.FrontToBack(SuperStructure.getInstance())
                        )
            )
        }
}