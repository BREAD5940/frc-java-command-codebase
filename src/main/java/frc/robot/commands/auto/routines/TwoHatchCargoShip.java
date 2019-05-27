package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.team5940.pantry.exparimental.command.CommandGroupBase;
import org.team5940.pantry.exparimental.command.SequentialCommandGroup;

import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.PipelinePreset;

/**
 * 2-hatch 1-cargo auto
 */
public class TwoHatchCargoShip extends SequentialCommandGroup {
	// private SendableCommandBase mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	public TwoHatchCargoShip(char arg1, char arg2) {
		this();
	}

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine.
	 * 
	 * @param side     to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public TwoHatchCargoShip(/* char startPos, char side */) {
		// HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		// String cStart = "hab" + startPos;

		/* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habR" + " to " + "caroMR"); //current trajectory from hashmap in Trajectories

		// addParallel(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE_PREP)); // move arm inside to prep state
		//		addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));
		//		addParallel(new LimeLight.setPipeline(PipelinePreset.k3dVision));
		//		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		addCommands(
				parallel(
						new LimeLight.SetLEDs(LimeLight.LEDMode.kON),
						new LimeLight.setPipeline(PipelinePreset.k3dVision),
						DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)));

		// addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		addCommands(new FollowVisionTargetTheSecond(4.3));

		addCommands(new RunIntake(-1, 0, 1));

		// addParallel(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));

		// spline over to the loading
		var rocketToLoading = Trajectories.generatedLGTrajectories.get("cargoMR to loadingR");
		addCommands(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		// addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));

		addCommands(new FollowVisionTargetTheSecond(5));

		addCommands(new RunIntake(-1, 0, 1));

		// addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));
		var loadingToRocketFar = Trajectories.generatedLGTrajectories.get("loadingR to rocketRF");
		addCommands(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal
		addCommands(new FollowVisionTargetTheSecond(4.3));

		addCommands(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));

	}

	// id functions

	/**
	 * identification function
	 * @return
	 *  the mBigCommandGroup of the function
	 */
	public CommandGroupBase getBigCommandGroup() {
		return this;
	}

	//not id functions

}
