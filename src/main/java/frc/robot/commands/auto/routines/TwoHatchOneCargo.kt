package frc.robot.commands.auto.routines

import java.util.ArrayList
import java.util.Arrays

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.*

import org.team5940.pantry.exparimental.command.ParallelCommandGroup
import org.team5940.pantry.exparimental.command.PrintCommand
import frc.robot.RobotConfig.auto.fieldPositions
import frc.robot.commands.auto.AutoMotion
import frc.robot.commands.auto.Trajectories
import frc.robot.commands.subsystems.drivetrain.DriveDistanceTheThird
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond
import frc.robot.commands.subsystems.superstructure.ArmMove
import frc.robot.commands.subsystems.superstructure.ElevatorMove
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.commands.subsystems.superstructure.RunIntake
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.DriveTrain.Gear
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode
import frc.robot.subsystems.LimeLight
import frc.robot.subsystems.LimeLight.PipelinePreset
import frc.robot.subsystems.superstructure.SuperStructure.iPosition
import org.team5940.pantry.exparimental.command.SequentialCommandGroup
import org.team5940.pantry.exparimental.command.WaitCommand

/**
 * 2-hatch 1-cargo auto
 */
class TwoHatchOneCargo
/**
 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine.
 * @param side to target (L or R)
 * @param startPos L M or R on the hab
 * @author Matthew Morley
 */
(/*char startPos, char side*/) : SequentialCommandGroup() {
    // private AutoCommandGroup mBigCommandGroup;
    var trajects = ArrayList<TimedTrajectory<Pose2dWithCurvature>>()
    var motions = ArrayList<AutoMotion>()

    constructor(arg1: Char, arg2: Char) : this() {}

    init {
        // HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
        // String cStart = "hab" + startPos;

        // addSequential(new InstantRunnable(() -> {
        // 	SuperStructure.getElevator().getMaster().configPeakOutputForward(0);
        // 	SuperStructure.getElevator().getMaster().configPeakOutputReverse(0);

        // 	SuperStructure.getInstance().getWrist().getMaster().configPeakOutputForward(0);
        // 	SuperStructure.getInstance().getWrist().getMaster().configPeakOutputReverse(0);

        // 	SuperStructure.getInstance().getElbow().getMaster().configPeakOutputForward(0);
        // 	SuperStructure.getInstance().getElbow().getMaster().configPeakOutputReverse(0);
        // }, true));

        // boolean doIntake = false;
        // boolean doVision = false;

        /* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
        val traject = Trajectories.generatedLGTrajectories["habR" + " to " + "rocketRF"] //current trajectory from hashmap in Trajectories

        addCommands(
                ParallelCommandGroup(
                        JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH),
                        LimeLight.SetLEDs(LimeLight.LEDMode.kON),
                        LimeLight.setPipeline(PipelinePreset.k3dVision),
                        DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)
                )
        )

        // addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
        addCommands(FollowVisionTargetTheSecond(3.8))

        // addSequential(new DriveDistanceTheThird(LengthKt.getFeet(0.4), false));

        // addSequential(new PrintCommand("GOT TO RUN INTAKE"));

        addCommands(RunIntake(-1.0, 0.0, 1.5))

        // addSequential(new PrintCommand("GOT TO BACKING UP"));

        // back up 3 feet
        // addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP)); // TODO fix this broken logic!

        // move the superstructure while driving backwards
        addCommands(ElevatorMove(iPosition.HATCH_GRAB_INSIDE.getElevator()).andThen(
                JankyGoToState(iPosition.HATCH_GRAB_INSIDE)).alongWith(
                DriveDistanceTheThird(3.feet, true)
        ))

        addCommands(PrintCommand("GOT TO next spline"))

        // // spline over to the rocket
        val rocketToLoading = Trajectories.generatedLGTrajectories["rocketRF to loadingR"]
        addCommands(DriveTrain.getInstance().followTrajectoryWithGear(rocketToLoading, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)) //drive to goal

        // // // addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));

        addCommands(FollowVisionTargetTheSecond(4.5))

        addCommands(RunIntake(1.0, 0.0, 1.0))

        // addSequential(new DriveDistanceTheThird(LengthKt.getFeet(1), false));

        // // // addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));
        // addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)); // move arm inside to prep state
        val loadingToRocketFar = Trajectories.generatedLGTrajectories["loadingR to rocketRF"]

        addCommands(
                DriveTrain.getInstance().followTrajectoryWithGear(
                        loadingToRocketFar, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false).alongWith(
                        ParallelCommandGroup(
                                WaitCommand(2.0),
                                ArmMove(iPosition.HATCH),
                                ElevatorMove(fieldPositions.hatchMiddleGoal))
                )
        )


        addCommands(DriveTrain.getInstance().followTrajectoryWithGear(loadingToRocketFar, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)) //drive to goal
        addCommands(FollowVisionTargetTheSecond(3.8))
        addCommands(RunIntake(-1.0, 0.0, 1.0))

        // // // addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));

    }

}
