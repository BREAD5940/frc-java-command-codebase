package frc.robot.commands.auto.routines

import java.util.ArrayList
import java.util.Arrays

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.*

import frc.robot.commands.auto.AutoMotion
import frc.robot.commands.auto.Trajectories
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond
import frc.robot.commands.subsystems.drivetrain.PIDDriveDistance
import frc.robot.commands.subsystems.drivetrain.SetGearCommand
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.commands.subsystems.superstructure.RunIntake
import frc.robot.lib.withTimeout
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.DriveTrain.Gear
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode
import frc.robot.subsystems.LimeLight
import frc.robot.subsystems.LimeLight.PipelinePreset
import frc.robot.subsystems.superstructure.SuperStructure.iPosition
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.team5940.pantry.exparimental.command.Command
import org.team5940.pantry.exparimental.command.SequentialCommandGroup

/**
 * 2-hatch 1-cargo auto
 */
class PlaceCargoFrontL (/* char startPos, char side */) : SequentialCommandGroup() {
    // private AutoCommandGroup mBigCommandGroup;
    var trajects = ArrayList<TimedTrajectory<Pose2dWithCurvature>>()
    var motions = ArrayList<AutoMotion>()

    // id functions

    /**
     * identification function
     * @return
     * the mBigCommandGroup of the function
     */
    val bigCommandGroup: Command
        get() = this

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
        // TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habL" + " to " + "cargoML"); //current trajectory from hashmap in Trajectories

        val traject = Trajectories.generateTrajectoryLowGear(Arrays.asList(
                // new Pose2d(LengthKt.getFeet(5.1), LengthKt.getFeet(17.684), Rotation2dKt.getDegree(0)),
                Pose2d(9.5.feet, 17.684.feet, 0.degree),
                Pose2d(15.1.feet, 14.434.feet, 0.degree)), false)

        addCommands(SetGearCommand(Gear.LOW))

        // addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(6), VelocityKt.getVelocity(LengthKt.getFeet(7)), false));
        addCommands(PIDDriveDistance(5.feet, 6.0))

        // addSequential(new DelayCommand(TimeUnitsKt.getSecond(1)).getWrappedValue());
        // addSequential(new WaitCommand(0.7));

        // addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)); // move arm inside to prep state
        addCommands(JankyGoToState(iPosition.HATCH_GRAB_INSIDE)) // move arm inside to prep state


        addCommands(LimeLight.SetLEDs(LimeLight.LEDMode.kON))
        addCommands(LimeLight.setPipeline(PipelinePreset.k3dVision))

        addCommands(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)) //drive to goal

        // addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
        addCommands(FollowVisionTargetTheSecond(5.0))

        // addSequential(new DriveDistanceTheThird(LengthKt.getFeet(0.4), false));

        // addSequential(new PrintCommand("GOT TO RUN INTAKE"));

        addCommands(RunIntake(-1.0, 0.0, 1.0))

        addCommands(PIDDriveDistance((-3.0).inch, 12.0) withTimeout 0.5.second)

        // addSequential(new DriveDistanceTheThird(LengthKt.getFeet(2), true));
    }

    //not id functions

}
