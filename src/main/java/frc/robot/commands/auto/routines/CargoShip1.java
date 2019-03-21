package frc.robot.commands.auto.routines;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.PlaceHatch;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * 2-hatch 1-cargo auto
 */
public class CargoShip1 extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	// public CargoShip1(char arg1, char arg2) {
	// this();
	// }

	/**
	 * Drive straight-ish forward to the left or right front of the cargo ship and place a hatch. That's literally it.
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public CargoShip1(char side) {

		boolean isLeft = side == 'L' || side == 'l';

		var fallOffTheHab = Arrays.asList(
				new Pose2d(LengthKt.getFeet(5.244),
						LengthKt.getFeet(13.536),
						Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(8.5),
						LengthKt.getFeet(14.58),
						Rotation2dKt.getDegree(0)));

		var toCargo1 = Arrays.asList(
				new Pose2d(LengthKt.getFeet(8.5),
						LengthKt.getFeet(14.58),
						Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(15.45),
						LengthKt.getFeet(14.49),
						Rotation2dKt.getDegree(0)));

		if (!isLeft)
			fallOffTheHab = Util.reflectTrajectory(fallOffTheHab);

		var p_fallOffTheHab = Trajectories.generateTrajectory(
				fallOffTheHab,
				Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		if (!isLeft)
			toCargo1 = Util.reflectTrajectory(toCargo1);

		var p_toCargo1 = Trajectories.generateTrajectory(
				toCargo1,
				Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		this.addSequential(DriveTrain.getInstance().followTrajectory(p_fallOffTheHab, TrajectoryTrackerMode.RAMSETE, true));
		addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		this.addSequential(DriveTrain.getInstance().followTrajectory(p_fallOffTheHab, TrajectoryTrackerMode.RAMSETE, false));
		addSequential(new PlaceHatch());

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
