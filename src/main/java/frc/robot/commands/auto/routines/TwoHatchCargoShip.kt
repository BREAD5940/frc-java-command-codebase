package frc.robot.commands.auto.routines

import java.util.ArrayList

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory

import frc.robot.commands.auto.AutoMotion
import frc.robot.commands.auto.Trajectories
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond
import frc.robot.commands.subsystems.superstructure.RunIntake
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.DriveTrain.Gear
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode
import frc.robot.subsystems.LimeLight
import frc.robot.subsystems.LimeLight.PipelinePreset
import org.team5940.pantry.exparimental.command.InstantCommand
import org.team5940.pantry.exparimental.command.ParallelCommandGroup
import org.team5940.pantry.exparimental.command.SequentialCommandGroup

/**
 * 2-hatch 1-cargo maybe
 *
 * @param side     to target (L or R)
 * @param startPos L M or R on the hab
 * @author Matthew Morley
 */class TwoHatchCargoShip

    : SequentialCommandGroup() {
    // private AutoCommandGroup mBigCommandGroup;
    var trajects = ArrayList<TimedTrajectory<Pose2dWithCurvature>>()
    var motions = ArrayList<AutoMotion>()

    // id functions

    /**
     * identification function
     * @return
     * the mBigCommandGroup of the function
     */
    val bigCommandGroup = InstantCommand()

    init {
        // HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
        // String cStart = "hab" + startPos;

        /* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
        val traject = Trajectories.generatedLGTrajectories["habR" + " to " + "caroMR"] //current trajectory from hashmap in Trajectories

        addCommands(
                ParallelCommandGroup(
                        (LimeLight.SetLEDs(LimeLight.LEDMode.kON)),
                        LimeLight.setPipeline(PipelinePreset.k3dVision),
                        DriveTrain.getInstance().followTrajectoryWithGear(
                                traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true) //drive to goal
                )
        )


//        // addParallel(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE_PREP)); // move arm inside to prep state
//        addParallel(LimeLight.SetLEDs(LimeLight.LEDMode.kON))
//        addParallel(LimeLight.setPipeline(PipelinePreset.k3dVision))
//        addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)) //drive to goal

        // addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
        addCommands(FollowVisionTargetTheSecond(4.3))

        addCommands(RunIntake(-1.0, 0.0, 1.0))

        // addParallel(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));

        // spline over to the loading
        val rocketToLoading = Trajectories.generatedLGTrajectories["cargoMR to loadingR"]
        addCommands(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)) //drive to goal

        // addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));

        addCommands(FollowVisionTargetTheSecond(5.0))

        addCommands(RunIntake(-1.0, 0.0, 1.0))

        // addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));
        val loadingToRocketFar = Trajectories.generatedLGTrajectories["loadingR to rocketRF"]
        addCommands(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)) //drive to goal
        addCommands(FollowVisionTargetTheSecond(4.3))

        addCommands(LimeLight.SetLEDs(LimeLight.LEDMode.kOFF))

    }

    //not id functions

}
