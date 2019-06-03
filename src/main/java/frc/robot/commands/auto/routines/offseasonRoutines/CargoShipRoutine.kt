package frc.robot.commands.auto.routines.offseasonRoutines

import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.command.WaitCommand
import frc.robot.RobotConfig
import frc.robot.commands.auto.Autonomous
import frc.robot.commands.auto.paths.TrajectoryFactory
import frc.robot.commands.auto.paths.TrajectoryWaypoints
import frc.robot.commands.auto.routines.AutoRoutine
import frc.robot.commands.subsystems.drivetrain.DrivePower
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.commands.subsystems.superstructure.PassThrough
import frc.robot.commands.subsystems.superstructure.RunIntake
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.superstructure.SuperStructure
//import org.ghrobotics.lib.commands.DelayCommand
//import org.ghrobotics.lib.commands.FalconCommand
//import org.ghrobotics.lib.commands.parallel
//import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

/**
 * Two hatch cargo ship auto. Can either start on the side and place two on the side,
 * or start on the front and do frontLeft and frontRight
 */
class CargoShipRoutine(private val mode: Mode, private val isLeft: Boolean) : AutoRoutine() {

    enum class Mode(
            val path1: TimedTrajectory<Pose2dWithCurvature>,
            val path2: TimedTrajectory<Pose2dWithCurvature>,
            val path3: TimedTrajectory<Pose2dWithCurvature>
    ) {
        SIDE(
                TrajectoryFactory.sideStartToCargoShipS1,
                TrajectoryFactory.cargoShipS1ToLoadingStation,
                TrajectoryFactory.loadingStationToCargoShipS2
        ),
        FRONT(
                TrajectoryFactory.centerStartToCargoShipFL,
                TrajectoryFactory.cargoShipFLToRightLoadingStation,
                TrajectoryFactory.loadingStationToCargoShipFR
        )
    }

    private val pathMirrored = {isLeft}

    val duration: Time
        get() = mode.path1.duration + mode.path2.duration + mode.path3.duration

    init {

        +InstantCommand{
            DriveTrain.getInstance().localization.reset(mode.path1.firstState.state.pose)
        }

        +parallel(
            followVisionAssistedTrajectory(mode.path1, pathMirrored, 4.feet, true),
            sequential(
                    WaitCommand(Math.max(mode.path1.duration.second - 3.5, 0.01)),
                    JankyGoToState(RobotConfig.auto.fieldPositions.hatchLowGoal, SuperStructure.iPosition.HATCH)
            )
        )

        val path2 = followVisionAssistedTrajectory(mode.path2, pathMirrored, 4.feet)

        // follow path2 while first outtaking, then close the intake, move the superstructure to the back and grab another hatch
        +parallel(
            path2,
            sequential(
                    RunIntake.grabHatchFor(0.5.second),
                    Intake.getInstance().clamp(),
                    PassThrough.FrontToBack(SuperStructure.getInstance()),
                    RunIntake.exhaustHatchFor(0.5.second)
            )
    )

        +relocalize(TrajectoryWaypoints.kLoadingStation, false, pathMirrored)

        +parallel(
            RunIntake.exhaustHatchFor(0.75.second),
            followVisionAssistedTrajectory(mode.path3, pathMirrored, 4.feet, true),
            sequential(
                    // stow for 2 seconds, then extend the big boi

                    JankyGoToState(RobotConfig.auto.fieldPositions.hatchLowGoal, SuperStructure.iPosition.HATCH)
//                executeFor(2.second, Superstructure.kStowedPosition),
//                Superstructure.kFrontHatchFromLoadingStation.withTimeout(3.second)
            )
        )

        +parallel(
            RunIntake.exhaustHatchFor(0.5.second),
            DrivePower(-0.3, 1.0)
        )
    }
}