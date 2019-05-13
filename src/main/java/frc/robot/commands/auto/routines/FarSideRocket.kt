package frc.robot.commands.auto.routines

import java.util.ArrayList
import java.util.Arrays

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity

import frc.robot.Robot
import frc.robot.RobotConfig.auto.fieldPositions
import frc.robot.commands.auto.AutoMotion
import frc.robot.commands.auto.Trajectories
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.lib.motion.Util
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.DriveTrain.Gear
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode
import frc.robot.subsystems.superstructure.SuperStructure.iPosition
import frc.robot.xboxmap
import org.team5940.pantry.exparimental.command.ParallelRaceGroup
import org.team5940.pantry.exparimental.command.SequentialCommandGroup
import org.team5940.pantry.exparimental.command.WaitUntilCommand

/**
 * 2-hatch 1-cargo auto
 */
class FarSideRocket
// public FarSideRocket(char arg1, char arg2) {
// this();
// }

/**
 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine.
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

        val kDefaultStartVelocity = 0.feet.velocity
        val kDefaultEndVelocity = 0.feet.velocity

        val kDefaultVelocityLow = 6.feet.velocity
        val kDefaultVelocityHigh = 9.feet.velocity

        val kDefaultAcceleration = 6.feet.acceleration

        var p_fallOffHab = Arrays.asList(
                Pose2d(
                        5.25.feet,
                        17.65.feet,
                        180.degree),
                Pose2d(
                        11.feet,
                        17.65.feet,
                        180.degree))

        var p_farSideRocketL = Arrays.asList(
                Pose2d(
                        11.feet,
                        17.65.feet,
                        180.degree),

                Pose2d(
                        19.787.feet,
                        21.655.feet,
                        (-140).degree),

                Pose2d(
                        24.118.feet,
                        23.658.feet,
                        150.0.degree))

        var p_halfWayToLoadingStationL = Arrays.asList(
                Pose2d(
                        22.888.feet,
                        24.157.feet,
                        (-30).degree),
                Pose2d(
                        22.2.feet,
                        20.feet,
                        180.degree),
                Pose2d(
                        9.403.feet,
                        19.827.feet,
                        90.degree))

        var p_toLoadingStation = Arrays.asList(
                Pose2d(
                        9.403.feet,
                        19.827.feet,
                        90.degree),
                Pose2d(
                        4.17.feet,
                        24.85.feet,
                        180.degree))

        if (!isLeft) {
            p_fallOffHab = Util.reflectTrajectory(p_fallOffHab)
            p_farSideRocketL = Util.reflectTrajectory(p_farSideRocketL)
            p_halfWayToLoadingStationL = Util.reflectTrajectory(p_halfWayToLoadingStationL)
            p_toLoadingStation = Util.reflectTrajectory(p_toLoadingStation)
        }

        // public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
        // List<? extends TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed, boolean optomizeSplines) {

        val t_fallOffHab = Trajectories.generateTrajectory(p_fallOffHab, Trajectories.kLowGearConstraints, kDefaultStartVelocity,
                5.feet.velocity, kDefaultVelocityLow, kDefaultAcceleration, true, true)

        val t_farSideRocketL = Trajectories.generateTrajectory(p_farSideRocketL, Trajectories.kLowGearConstraints, 5.feet.velocity,
                0.feet.velocity, 7.feet.velocity, kDefaultAcceleration, true, true)

        val t_halfWayToLoadingStationL = Trajectories.generateTrajectory(p_halfWayToLoadingStationL, Trajectories.kLowGearConstraints, 0.feet.velocity,
                0.feet.velocity, 7.feet.velocity, kDefaultAcceleration, false, true)

        val t_toLoadingStationL = Trajectories.generateTrajectory(p_toLoadingStation, Trajectories.kLowGearConstraints, 0.feet.velocity,
                0.feet.velocity, 7.feet.velocity, kDefaultAcceleration, false, true)

        addCommands(DriveTrain.getInstance().followTrajectoryWithGear(t_fallOffHab, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)) // fall off the hab

        addCommands(JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP)
                .andThen(DriveTrain.getInstance().followTrajectoryWithGear(
                        t_farSideRocketL, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false))) // keep going over to the far side of the rocket


        addCommands(JankyGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH))

        addCommands(ParallelRaceGroup(WaitUntilCommand { Robot.m_oi.primary.getRawButton(xboxmap.Buttons.A_BUTTON) }, TeleopCommands()))

        // addSequential(new FollowVisionTargetTheSecond(3.8));
        // addSequential(new RunIntake(-1, 0, 1));

        // addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_halfWayToLoadingStationL, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // nyoom off to the side
        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_toLoadingStationL, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); // go to the loading station
        // addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
        // addSequential(new FollowVisionTargetTheSecond(4.5));
        // addSequential(new PIDDriveDistance(0.5, 4, /* timeout */ 0.5));
        // addSequential(new RunIntake(1, 0, 1));
        // addSequential(new PIDDriveDistance(-3, 12, /* timeout */ 1));

    }

}
