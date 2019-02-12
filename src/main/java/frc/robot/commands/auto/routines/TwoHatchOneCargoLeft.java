package frc.robot.commands.auto.routines;

import org.ghrobotics.lib.commands.DelayCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;

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
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, true)); //drive to goal
		// this.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH,true).getBigCommandGroup()); //do a motion
		this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		/* Move from cargo ship to loading station on the same side */
		cStart = "cargoM" + side;
		cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "loading" + side); //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_HATCH,false).getBigCommandGroup()); //do a motion
		this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		/* Go right up to the cargo ship from the loading station */
		cStart = "loading" + side;
		cPiece = HeldPiece.HATCH;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); /*FIXME this path doesn't exist*/ //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal

		// turn 90 degrees to face the goal
		this.addSequential(new TurnInPlace(90f, true)); // TODO check the angle
		this.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH, false).getBigCommandGroup()); //move the intake for hatch placement

		/* Get a cargo from the depot */
		cStart = "cargo" + side + '1';
		cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "depot" + side); //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue()); // TODO run a pickup script

		/* Go from depot to cargo ship 2 */
		cStart = "depot" + side;
		cPiece = HeldPiece.CARGO;
		//correct me if im wrong, but why would we go to the second cargo? it doesn't have a hatch. 
		//the cargo'd fall out at the end of auto unless we used a null hatch, which has been argued against
		//The only reason I can think of is so the elevator doesn't have to go up, but that doesn't help us if we don't get the pts anyway
		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '2'); //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		this.addSequential(new TurnInPlace(90f, true)); // TODO check the angle
		//FIXME this would have to raise the elevator
		this.addSequential(new AutoMotion(GoalHeight.OVER, GoalType.CARGO_CARGO, false).getBigCommandGroup()); //deposit cargo

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
