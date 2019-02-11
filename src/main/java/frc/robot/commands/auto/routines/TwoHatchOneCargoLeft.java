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
 * Creates an AutoMotion and drive plan based on the inputted params. Will
 * probably be used only in sandstorm. (and yes, this is basically 2018's auto
 * selector, but slightly different)
 */
public class TwoHatchOneCargoLeft extends AutoCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow.
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 */
	//TODO add support for 'yeet in a circle while moving'
	public TwoHatchOneCargoLeft(char startPos, char side) {
		HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		String cStart = "hab" + startPos;

		/* Get a trajectory to move to the cargo ship */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedTrajectories.get(cStart + " to " + "cargoM" + side); //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, true)); //drive to goal
		// this.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH,true).getBigCommandGroup()); //do a motion
		this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		/* Move from cargo ship to loading station on the same side */
		cStart = "cargoM" + side;
		cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedTrajectories.get(cStart + " to " + "loading" + side); //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_HATCH,false).getBigCommandGroup()); //do a motion
		this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		/* Go right up to the cargo ship from the loading station */
		cStart = "loading" + side;
		cPiece = HeldPiece.HATCH;

		traject = Trajectories.generatedTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal

		// turn 90 degrees to face the goal
		this.addSequential(new TurnInPlace(90f, true)); // TODO check the angle
		this.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH, false).getBigCommandGroup()); //move the intake for hatch placement

		/* Get a cargo from the depot */
		cStart = "cargo" + side + '1';
		cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedTrajectories.get(cStart + " to " + "depot" + side); //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue()); // TODO run a pickup script

		/* Go from depot to cargo ship 2 */
		cStart = "depot" + side;
		cPiece = HeldPiece.CARGO;

		traject = Trajectories.generatedTrajectories.get(cStart + " to " + "cargo" + side + '2'); //current trajectory from hashmap in Trajectorie
		this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		this.addSequential(new TurnInPlace(90f, true)); // TODO check the angle

		// TODO Deposit cargo here

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
