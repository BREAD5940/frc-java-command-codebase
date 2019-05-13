package frc.robot.commands.auto.routines

import java.util.ArrayList
import java.util.Arrays

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.*

import frc.robot.RobotConfig.auto.fieldPositions
import frc.robot.commands.auto.AutoMotion
import frc.robot.commands.auto.Trajectories
import frc.robot.commands.auto.groups.PlaceHatch
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.lib.motion.Util
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode
import frc.robot.subsystems.superstructure.SuperStructure.iPosition
import org.team5940.pantry.exparimental.command.SequentialCommandGroup

/**
 * 2-hatch 1-cargo auto
 */
class CargoShip1
// public CargoShip1(char arg1, char arg2) {
// this();
// }

/**
 * Drive straight-ish forward to the left or right front of the cargo ship and place a hatch. That's literally it.
 * @param side to target (L or R)
 * @param startPos L M or R on the hab
 * @author Matthew Morley
 */
(side: Char) : SequentialCommandGroup() {
    // private AutoCommandGroup mBigCommandGroup;
    var trajects = ArrayList<TimedTrajectory<Pose2dWithCurvature>>()
    var motions = ArrayList<AutoMotion>()

    init {

        val isLeft = side == 'L' || side == 'l'

        var fallOffTheHab = Arrays.asList(
                Pose2d(5.244.feet,
                        13.536.feet,
                        0.degree),
                Pose2d(8.5.feet,
                        14.58.feet,
                        0.degree))

        var toCargo1 = Arrays.asList(
                Pose2d(8.5.feet,
                        14.58.feet,
                        0.degree),
                Pose2d(15.45.feet,
                        14.49.feet,
                        0.degree))

        if (!isLeft)
            fallOffTheHab = Util.reflectTrajectory(fallOffTheHab)

        val p_fallOffTheHab = Trajectories.generateTrajectory(
                fallOffTheHab,
                Trajectories.kLowGearConstraints,
                0.0.feet.velocity,
                0.0.feet.velocity,
                6.0.feet.velocity,
                8.0.feet.acceleration,
                false,
                true)

        if (!isLeft)
            toCargo1 = Util.reflectTrajectory(toCargo1)

        val p_toCargo1 = Trajectories.generateTrajectory(
                toCargo1,
                Trajectories.kLowGearConstraints,
                0.0.feet.velocity,
                0.0.feet.velocity,
                6.0.feet.velocity,
                8.0.feet.acceleration,
                false,
                true)

        addCommands(DriveTrain.getInstance().followTrajectory(p_fallOffTheHab, TrajectoryTrackerMode.RAMSETE, true))
        addCommands(JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH).andThen(
                DriveTrain.getInstance().followTrajectory(p_fallOffTheHab, TrajectoryTrackerMode.RAMSETE, false)))
        addCommands(PlaceHatch())

    }

    //not id functions

}
