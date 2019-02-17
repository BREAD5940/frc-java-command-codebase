package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.subsystems.drivetrain.TurnInPlace;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

/**
 * 2-hatch 1-cargo auto
 */
//FIXME why is this specified as left if it also takes an input for side?
public class TwoHatchOneCargoLeft extends AutoCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow.
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 */
	public TwoHatchOneCargoLeft(char startPos, char side) {
		HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		String cStart = "hab" + startPos;

		/* Get a trajectory to move to the cargo ship */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargoM" + side); //current trajectory from hashmap in Trajectorie
		AutoMotion motion = new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH, true, false);
		trajects.add(traject);
		motions.add(motion);
		this.addParallel(motion.getPrepCommand());
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, true)); //drive to goal
		this.addSequential(motion.getBigCommandGroup()); //do a motion
		// this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		/* Move from middle of cargo ship to loading station on the same side to pick up a hatch */
		cStart = "cargoM" + side;
		cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "loading" + side); //current trajectory from hashmap in Trajectorie
		motion = new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_HATCH, false);
		trajects.add(traject);
		motions.add(motion);
		this.addParallel(motion.getPrepCommand());
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		this.addSequential(motion.getBigCommandGroup()); //do a motion
		// this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		/* Go right up to the cargo ship from the loading station */
		cStart = "loading" + side;
		cPiece = HeldPiece.HATCH;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		motion = new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH, false);
		trajects.add(traject);
		motions.add(motion);
		this.addParallel(motion.getPrepCommand());
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal

		// turn 90 degrees to face the goal
		this.addSequential(new TurnInPlace(Trajectories.locations.get("cargo" + side + '1').component2(), true)); // TODO check the angle math here! 
		this.addSequential(motion.getBigCommandGroup()); //move the intake for hatch placement

		/* Go from cargo side 1 to the depot */
		cStart = "cargo" + side + '1';
		cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "depot" + side); //current trajectory from hashmap in Trajectorie
		motion = new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_CARGO, false);
		trajects.add(traject);
		motions.add(motion);
		this.addParallel(motion.getPrepCommand());
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		this.addSequential(motion.getBigCommandGroup());
		// this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		/* Go from depot to cargo ship ~~2~~ 1 darnit you're right. Thanks 10pm me */
		cStart = "depot" + side;
		cPiece = HeldPiece.CARGO;
		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		motion = new AutoMotion(GoalHeight.OVER, GoalType.CARGO_CARGO, false);
		trajects.add(traject);
		motions.add(motion);
		this.addParallel(motion.getPrepCommand());
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		this.addSequential(new TurnInPlace(Rotation2dKt.getDegree(90), true)); // TODO check the angle
		//FIXME this would have to raise the elevator
		this.addSequential(motion.getBigCommandGroup()); //deposit cargo

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
