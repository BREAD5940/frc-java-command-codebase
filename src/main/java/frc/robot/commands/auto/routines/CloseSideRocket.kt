package frc.robot.commands.auto.routines

import java.util.ArrayList
import java.util.Arrays
import java.util.function.Supplier

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
import frc.robot.lib.alongWith
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
class CloseSideRocket (side: Char) : SequentialCommandGroup() {
    // private AutoCommandGroup mBigCommandGroup;
    var trajects = ArrayList<TimedTrajectory<Pose2dWithCurvature>>()
    var motions = ArrayList<AutoMotion>()


    init {

        val isLeft = side == 'L' || side == 'l'

        val kDefaultStartVelocity = 0.feet.velocity
        val kDefaultEndVelocity = 0.feet.velocity

        val kDefaultVelocityLow = 5.feet.velocity
        val kDefaultVelocityHigh = 9.feet.velocity

        val kDefaultAcceleration = 6.feet.acceleration

        var rocketC = Arrays.asList(
                Pose2d(5.331.feet,
                        17.707.feet,
                        0.degree),
                Pose2d(9.039.feet,
                        19.756.feet,
                        44.313.degree)
                // new Pose2d(LengthKt.getFeet(15.712),
                // 	LengthKt.getFeet(24.817),
                // 	Rotation2dKt.getDegree(30))
        )

        if (!isLeft)
            rocketC = Util.reflectTrajectory(rocketC)

        val p_rocketC = Trajectories.generateTrajectory(
                rocketC,
                Trajectories.kLowGearConstraints,
                0.0.feet.velocity,
                6.0.feet.velocity,
                6.0.feet.velocity,
                8.0.feet.acceleration,
                false,
                true)

        var rocketCPart2 = Arrays.asList(
                // new Pose2d(LengthKt.getFeet(5.331),
                // 	LengthKt.getFeet(17.707),
                // 	Rotation2dKt.getDegree(0)),
                Pose2d(9.039.feet,
                        19.756.feet,
                        44.313.degree),
                Pose2d(14.707.feet,
                        24.407.feet,
                        30.degree))

        if (!isLeft)
            rocketCPart2 = Util.reflectTrajectory(rocketCPart2)

        val p_rocketCPart2 = Trajectories.generateTrajectory(
                rocketCPart2,
                Trajectories.kLowGearConstraints,
                6.0.feet.velocity,
                0.0.feet.velocity,
                6.0.feet.velocity,
                8.0.feet.acceleration,
                false,
                true)

        // if (!isLeft) {

        // public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
        // List<? extends TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed, boolean optomizeSplines) {

        // var t_hab = Trajectories.generateTrajectory(p_hab, Trajectories.kLowGearConstraints, kDefaultStartVelocity,
        // 		VelocityKt.getVelocity(LengthKt.getFeet(/*7*/ 4)), VelocityKt.getVelocity(LengthKt.getFeet(/*7*/ 6)), kDefaultAcceleration, false, true);

        // var t_toPlaceHatch = Trajectories.generateTrajectory(p_toHatchPlace, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(/*7*/ 4)),
        // 		kDefaultEndVelocity, VelocityKt.getVelocity(LengthKt.getFeet(/*7*/ 5)), kDefaultAcceleration, false, true);

        // var t_halfWayToLoadingStationL = Trajectories.generateTrajectory(p_halfWayToLoadingStationL, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(0)),
        // 		VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(6)), kDefaultAcceleration, true, true);

        // var t_toLoadingStation = Trajectories.generateTrajectory(p_toLoadingStation, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(0)),
        // 		VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(6)), kDefaultAcceleration, false, true);

        addCommands(DriveTrain.getInstance().followTrajectoryWithGear(p_rocketC, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)) // keep going over to the far side of the rocket
        addCommands(JankyGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH) alongWith DriveTrain.getInstance().followTrajectoryWithGear(p_rocketCPart2, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)) // keep going over to the far side of the rocket

        val checker = { java.lang.Boolean.valueOf(Robot.m_oi.primary.getRawButton(xboxmap.Buttons.A_BUTTON)) }

        addCommands(ParallelRaceGroup(WaitUntilCommand(checker), TeleopCommands()))

        // addSequential(new DrivePower(-0.4, 0.5));

        addCommands(JankyGoToState(iPosition.HATCH_GRAB_INSIDE))

        // addSequential(new PlaceHatch());

        // addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
        // CommandGroup waitForABit = new CommandGroup();
        // waitForABit.addSequential(new WaitCommand("yes", 4));
        // waitForABit.addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
        // addParallel(waitForABit);

        // addSequential(new WaitCommand(0.2));

        // addSequential(new FollowVisionTargetTheSecond(3.5));
        // addSequential(new DriveDistanceTheThird(LengthKt.getInch(6), false));
        // addSequential(new RunIntake(-1, 0, 1));

        // addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_halfWayToLoadingStationL, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // nyoom off to the side
        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_toLoadingStation, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); // go to the loading station
        // // addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
        // addSequential(new FollowVisionTargetTheSecond(4.5));
        // addSequential(new PIDDriveDistance(0.5, 4, /* timeout */ 0.5));
        // addSequential(new RunIntake(1, 0, 1));
        // addSequential(new PIDDriveDistance(-5, 12, /* timeout */ 1));

    }

    //not id functions

}
