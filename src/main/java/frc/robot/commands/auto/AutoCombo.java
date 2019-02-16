package frc.robot.commands.auto;

import org.ghrobotics.lib.commands.DelayCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;

import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.routines.TwoHatchOneCargoLeft;
import frc.robot.commands.subsystems.drivetrain.TurnInPlace;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

/**
 * Creates an AutoMotion and drive plan based on the inputted params. Will
 * probably be used only in sandstorm. 
 * 
 * @author Jocelyn McHugo
 */
public class AutoCombo {
	private AutoCommandGroup mBigCommandGroup;

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow.
	 * @param startingPiece the game piece that the robot starts with (for example, a hatch)
	 * @param wpKeys the keys for the waypoints (defined in Trajectories) for toe robot to go to.
	 */
	//TODO add support for 'yeet in a circle while moving'
	public AutoCombo(String start, char centerSide) {
		HeldPiece cPiece = HeldPiece.HATCH;
		String cStart = start;
		char side;
		if (cStart.equals("habM")) {
			side = centerSide;
		} else {
			side = cStart.charAt(cStart.length() - 1);
		}
		cStart = "hab";

		mBigCommandGroup=new TwoHatchOneCargoLeft(start.charAt(start.length()-1),side);
		// TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargoM" + side); //current trajectory from hashmap in Trajectorie
		// AutoMotion motion = new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH, true);
		// this.mBigCommandGroup.addParallel(motion.getPrepCommand());
		// this.mBigCommandGroup.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, true)); //drive to goal
		// this.mBigCommandGroup.addSequential(motion.getBigCommandGroup()); //do a motion
		// // this.mBigCommandGroup.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		// cStart = "cargoM" + side;
		// cPiece = HeldPiece.NONE;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "loading" + side); //current trajectory from hashmap in Trajectorie
		// motion = new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_HATCH,false);
		// this.mBigCommandGroup.addParallel(motion.getPrepCommand());
		// this.mBigCommandGroup.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.mBigCommandGroup.addSequential(motion.getBigCommandGroup()); //do a motion
		// // this.mBigCommandGroup.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		// cStart = "loading" + side;
		// cPiece = HeldPiece.HATCH;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "yeet" + side); //current trajectory from hashmap in Trajectorie
		// this.mBigCommandGroup.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.mBigCommandGroup.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		// // this.mBigCommandGroup.addSequential(new YeetInACircleWhileMoving(true));
		// this.mBigCommandGroup.addSequential(new TurnInPlace(Rotation2dKt.getDegree(180), false));

		// cStart = "pyeet" + side;
		// cPiece = HeldPiece.HATCH;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		// this.mBigCommandGroup.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.mBigCommandGroup.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		// this.mBigCommandGroup.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH, false).getBigCommandGroup()); //do a motion
		// cStart = "cargo" + side + '1';
		// cPiece = HeldPiece.NONE;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "depot" + side); //current trajectory from hashmap in Trajectorie
		// this.mBigCommandGroup.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.mBigCommandGroup.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		// // this.mBigCommandGroup.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_CARGO,true).getBigCommandGroup()); //do a motion
		// cStart = "depot" + side;
		// cPiece = HeldPiece.CARGO;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		// this.mBigCommandGroup.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.mBigCommandGroup.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		// // this.mBigCommandGroup.addSequential(new AutoMotion(GoalHeight.LOW, GoalType.CARGO_CARGO,false).getBigCommandGroup()); //do a motion
		// cStart = "cargo" + side + '1';
		// cPiece = HeldPiece.NONE;

	}

	// id functions

	/**
	 * identification function
	 * @return
	 *  the mBigCommandGroup of the function
	 */
	public AutoCommandGroup getBigCommandGroup() {
		return this.mBigCommandGroup;
	}

	//not id functions

}
