package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.InstantRunnable;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceTheSecond;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.PipelinePreset;
import frc.robot.subsystems.superstructure.SuperStructure;

/**
 * 2-hatch 1-cargo auto
 */
public class TwoHatchOneCargo extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	public TwoHatchOneCargo(char arg1, char arg2) {
		this();
	}

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine. 
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public TwoHatchOneCargo(/*char startPos, char side*/) {
		// HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		// String cStart = "hab" + startPos;

		addSequential(new InstantRunnable(() -> {
			SuperStructure.getElevator().getMaster().configPeakOutputForward(0);
			SuperStructure.getElevator().getMaster().configPeakOutputReverse(0);

			SuperStructure.getInstance().getWrist().getMaster().configPeakOutputForward(0);
			SuperStructure.getInstance().getWrist().getMaster().configPeakOutputReverse(0);

			SuperStructure.getInstance().getElbow().getMaster().configPeakOutputForward(0);
			SuperStructure.getInstance().getElbow().getMaster().configPeakOutputReverse(0);
		}, true));

		boolean doIntake = false;
		boolean doVision = false;

		/* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habR" + " to " + "rocketRF"); //current trajectory from hashmap in Trajectories

		// addParallel(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE_PREP)); // move arm inside to prep state
		addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));
		addParallel(new LimeLight.setPipeline(PipelinePreset.k3dVision));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		// addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		addSequential(new FollowVisionTargetTheSecond(4.3));

		addSequential(new RunIntake(-1, 0, 1));

		// back up 3 feet
		// addParallel(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(3), true));

		// spline over to the rocket
		var rocketToLoading = Trajectories.generatedLGTrajectories.get("rocketRF to loadingR");
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		// addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));

		addSequential(new FollowVisionTargetTheSecond(5));

		addSequential(new RunIntake(-1, 0, 1));

		// addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));
		var loadingToRocketFar = Trajectories.generatedLGTrajectories.get("loadingR to rocketRF");
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal
		addSequential(new FollowVisionTargetTheSecond(4.3));

		addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));

	}

	// id functions

	/**
	 * identification function
	 * @return
	 *  the mBigCommandGroup of the function
	 */
	public AutoCommandGroup getBigCommandGroup() {
		return this;
	}

	//not id functions

}
