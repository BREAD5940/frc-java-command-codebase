package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.actions.SetTempPoseFromVisionTarget;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.drivetrain.SplineToVisionTarget;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * 2-hatch 1-cargo auto
 */
public class TwoHatchOneCargo extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine. 
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public TwoHatchOneCargo(char startPos, char side) {
		HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		String cStart = "hab" + startPos;

		/* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habL" + " to " + "rocketLF"); //current trajectory from hashmap in Trajectorie
		addParallel(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE_PREP)); // move arm inside to prep state
		this.addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal
		// addSequential(new SetTempPoseFromVisionTarget(this, PoseStorage.POSE1, false)); // set this' temp pose from vision target
		addParallel(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE.elevator.plus(iPosition.kOffsetFromL1toL2), iPosition.HATCH_SLAM_ROCKET_INSIDE.jointAngles));
		addSequential(new SplineToVisionTarget(this.getPoseStorage1(), 6.5));
		addSequential(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE), 1);
		addSequential(new RunIntake(-1, 0, 1));

		/* Move from middle of cargo ship to loading station on the same side to pick up a hatch */
		// cStart = "cargoM" + side;
		// cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedHGTrajectories.get("rocketLF" + " to " + "loading" + side); //current trajectory from hashmap in Trajectorie
		this.addParallel(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		this.addSequential(DriveTrain.getInstance().followTrajectory(Trajectories.reverse3FeetLowGear, TrajectoryTrackerMode.RAMSETE, false)); //back up mega fast to clear rocket
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		addSequential(new SetTempPoseFromVisionTarget(this, PoseStorage.POSE1, false));
		addSequential(new SplineToVisionTarget(this.getPoseStorage1(), 6.5));

		// /* Go right up to the cargo ship from the loading station */
		// cStart = "loading" + side;
		// cPiece = HeldPiece.HATCH;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		// motion = new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH, false);
		// trajects.add(traject);
		// motions.add(motion);
		// this.addParallel(motion.getPrepCommand());
		// this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal

		// // turn 90 degrees to face the goal
		// this.addSequential(new TurnInPlace(Trajectories.locations.get("cargo" + side + '1').component2(), true)); // TODO check the angle math here! 
		// this.addSequential(motion.getBigCommandGroup()); //move the intake for hatch placement

		// /* Go from cargo side 1 to the depot */
		// cStart = "cargo" + side + '1';
		// cPiece = HeldPiece.NONE;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "depot" + side); //current trajectory from hashmap in Trajectorie
		// motion = new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_CARGO, false);
		// trajects.add(traject);
		// motions.add(motion);
		// this.addParallel(motion.getPrepCommand());
		// this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.addSequential(motion.getBigCommandGroup());
		// // this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		// /* Go from depot to cargo ship ~~2~~ 1 darnit you're right. Thanks 10pm me */
		// cStart = "depot" + side;
		// cPiece = HeldPiece.CARGO;
		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		// motion = new AutoMotion(GoalHeight.OVER, GoalType.CARGO_CARGO, false);
		// trajects.add(traject);
		// motions.add(motion);
		// this.addParallel(motion.getPrepCommand());
		// this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.addSequential(new TurnInPlace(Rotation2dKt.getDegree(90), true)); // TODO check the angle
		// //FIXME this would have to raise the elevator
		// this.addSequential(motion.getBigCommandGroup()); //deposit cargo

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
