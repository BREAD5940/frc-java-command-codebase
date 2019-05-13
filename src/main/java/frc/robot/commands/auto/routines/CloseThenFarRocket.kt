package frc.robot.commands.auto.routines

import java.util.Arrays

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.*

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.robot.RobotConfig.auto.fieldPositions
import frc.robot.commands.auto.Trajectories
import frc.robot.commands.subsystems.drivetrain.TurnToFaceVisionTarget
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.lib.andThen
import frc.robot.lib.motion.Util
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode
import frc.robot.subsystems.superstructure.SuperStructure.iPosition
import org.team5940.pantry.exparimental.command.SequentialCommandGroup

class CloseThenFarRocket
/**
 * Semi-auto routine for placing on the far rocet followed by the close one;
 */
(side: Char) : SequentialCommandGroup() {
    init {

        val isLeft = side == 'L' || side == 'l'

        var fallOFfHab = Arrays.asList(
                Pose2d(5.175.feet,
                        17.689.feet,
                        0.degree),
                Pose2d(8.feet,
                        18.5.feet,
                        40.degree)

        )

        var floorToRocketC = Arrays.asList(
                Pose2d(8.0.feet,
                        18.5.feet,
                        40.degree),
                Pose2d(14.14.feet,
                        24.5.feet,
                        20.degree))

        val rocketCToLoading = Arrays.asList(
                Pose2d(15.7.feet,
                        24.37.feet,
                        32.degree),
                Pose2d(5.572.feet,
                        23.017.feet,
                        (-10).degree),
                Pose2d(5.feet,
                        24.918.feet,
                        180.degree),
                Pose2d(6.5.feet,
                        24.882.feet,
                        180.degree))

        // var loadingToRocketF = Arrays.asList(
        // 		new Pose2d(LengthKt.getFeet(1.465),
        // 				LengthKt.getFeet(24.871),
        // 				Rotation2dKt.getDegree(180)),
        // 		new Pose2d(LengthKt.getFeet(19.871),
        // 				LengthKt.getFeet(22.601),
        // 				Rotation2dKt.getDegree(-154.855)),
        // 		new Pose2d(LengthKt.getFeet(23.772),
        // 				LengthKt.getFeet(23.551),
        // 				Rotation2dKt.getDegree(148)));

        // var rocketFtoLoading = Arrays.asList(
        // 		new Pose2d(LengthKt.getFeet(22.438),
        // 				LengthKt.getFeet(24.194),
        // 				Rotation2dKt.getDegree(148)),
        // 		new Pose2d(LengthKt.getFeet(21.338),
        // 				LengthKt.getFeet(22.673),
        // 				Rotation2dKt.getDegree(0)),
        // 		new Pose2d(LengthKt.getFeet(6.023),
        // 				LengthKt.getFeet(22.892),
        // 				Rotation2dKt.getDegree(-4)),
        // 		new Pose2d(LengthKt.getFeet(4.5),
        // 				LengthKt.getFeet(24.8),
        // 				Rotation2dKt.getDegree(180)));

        if (!isLeft) {
            fallOFfHab = Util.reflectTrajectory(fallOFfHab)
            floorToRocketC = Util.reflectTrajectory(floorToRocketC)
            // rocketCToLoading = Util.reflectTrajectory(rocketCToLoading);
            // loadingToRocketF = Util.reflectTrajectory(loadingToRocketF);
            // rocketFtoLoading = Util.reflectTrajectory(rocketFtoLoading);
        }

        val t_fallOFfHab = Trajectories.generateTrajectory(fallOFfHab, Trajectories.kLowGearConstraints,

                0.0.feet.velocity,
                6.0.feet.velocity,
                7.0.feet.velocity,
                8.0.feet.acceleration,
                false,
                true)

        val t_floorToRocketC = Trajectories.generateTrajectory(floorToRocketC, Trajectories.kLowGearConstraints,

                7.0.feet.velocity,
                0.0.feet.velocity,
                6.5.feet.velocity,
                8.0.feet.acceleration,
                false,
                true)

        var rocketCtoTurn = Arrays.asList(
                Pose2d(15.435.feet,
                        24.426.feet,
                        30.degree),
                Pose2d(11.136.feet,
                        20.feet,
                        90.degree))

        if (!isLeft)
            rocketCtoTurn = Util.reflectTrajectory(rocketCtoTurn)

        val p_rocketCtoTurn = Trajectories.generateTrajectory(
                rocketCtoTurn,
                Trajectories.kLowGearConstraints,
                0.0.feet.velocity,
                0.0.feet.velocity,
                6.0.feet.velocity,
                8.0.feet.acceleration,
                true,
                true)

        var turnToLoading = Arrays.asList(
                Pose2d(11.136.feet,
                        20.feet,
                        90.degree),
                Pose2d(5.feet,
                        24.782.feet,
                        180.degree))

        if (!isLeft)
            turnToLoading = Util.reflectTrajectory(turnToLoading)

        val p_turnToLoading = Trajectories.generateTrajectory(
                turnToLoading,
                Trajectories.kLowGearConstraints,
                0.0.feet.velocity,
                0.0.feet.velocity,
                6.0.feet.velocity,
                8.0.feet.acceleration,
                false,
                true)

        var loadingToRocketF = Arrays.asList(
                Pose2d(1.811.feet,
                        24.925.feet,
                        180.degree),
                Pose2d(15.141.feet,
                        23.855.feet,
                        168.462.degree),
                Pose2d(19.456.feet,
                        22.946.feet,
                        (-174.87).degree),
                Pose2d(23.685.feet,
                        23.748.feet,
                        147.752.degree))

        if (!isLeft)
            loadingToRocketF = Util.reflectTrajectory(loadingToRocketF)

        val p_loadingToRocketF = Trajectories.generateTrajectory(
                turnToLoading,
                Trajectories.kLowGearConstraints,
                0.0.feet.velocity,
                0.0.feet.velocity,
                6.0.feet.velocity,
                8.0.feet.acceleration,
                true,
                true)

        var rocketFToLoadingIsh = Arrays.asList(
                Pose2d(22.351.feet,
                        24.46.feet,
                        148.degree),
                Pose2d(5.78.feet,
                        22.678.feet,
                        (-21.852).degree),
                Pose2d(9.074.feet,
                        24.835.feet,
                        180.degree))

        if (!isLeft)
            rocketFToLoadingIsh = Util.reflectTrajectory(rocketFToLoadingIsh)

        val p_rocketFToLoadingIsh = Trajectories.generateTrajectory(
                rocketFToLoadingIsh,
                Trajectories.kLowGearConstraints,
                0.0.feet.velocity,
                0.0.feet.velocity,
                6.0.feet.velocity,
                8.0.feet.acceleration,
                true,
                true)

        addCommands(DriveTrain.getInstance().followTrajectoryWithGear(t_fallOFfHab, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true))
        addCommands(JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH) andThen
                DriveTrain.getInstance().followTrajectoryWithGear(t_floorToRocketC, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, false))

        addCommands(TurnToFaceVisionTarget())

        // addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(35), VelocityKt.getVelocity(LengthKt.getFeet(2))));

        // addSequential(new ParallelRaceGroup(() -> (Robot.m_oi.getPrimary().getRawButton(xboxmap.Buttons.A_BUTTON)), new TeleopCommands()));

        // addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));

        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(p_rocketCtoTurn, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));

        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(p_turnToLoading, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, false));

        // addSequential(new ParallelRaceGroup(() -> (Robot.m_oi.getPrimary().getRawButton(xboxmap.Buttons.A_BUTTON)) , new TeleopCommands()));

        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(p_loadingToRocketF, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));

        // addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));

        // addSequential(new ParallelRaceGroup(() -> (Robot.m_oi.getPrimary().getRawButton(xboxmap.Buttons.A_BUTTON)) , new TeleopCommands()));

        // addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(20), VelocityKt.getVelocity(LengthKt.getFeet(3))));

        // addSequential(new GrabHatch());

        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_loadingToRocketF, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));
        // addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));

        // addSequential(new PlaceHatch());

        // addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));
        // addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_rocketFtoLoading, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));

    }
}
