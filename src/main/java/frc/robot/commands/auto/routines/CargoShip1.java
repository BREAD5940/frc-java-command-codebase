package frc.robot.commands.auto.routines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.drivetrain.PIDDriveDistance;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
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
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine. 
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public CargoShip1(char side) {

		boolean isLeft = side == 'L' || side == 'l';

		final Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
		final Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));

		final Velocity<Length> kDefaultVelocityLow = VelocityKt.getVelocity(LengthKt.getFeet(4));
		final Velocity<Length> kDefaultVelocityHigh = VelocityKt.getVelocity(LengthKt.getFeet(9));

		final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(6));

		List<Pose2d> p_toLeft1 = Arrays.asList(
				new Pose2d(LengthKt.getFeet(5.2), LengthKt.getFeet(17.6), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(9.802), LengthKt.getFeet(17.564), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(17.376), LengthKt.getFeet(21.539), Rotation2dKt.getDegree(25.868)),
				new Pose2d(LengthKt.getFeet(21.7), LengthKt.getFeet(18.847), Rotation2dKt.getDegree(-90)),
				new Pose2d(LengthKt.getFeet(21.7), LengthKt.getFeet(16.619), Rotation2dKt.getDegree(-90)));

		List<Pose2d> p_halfWayToLoadingStationL = Arrays.asList(
				new Pose2d(LengthKt.getFeet(21.7), LengthKt.getFeet(16.6), Rotation2dKt.getDegree(-90)),
				new Pose2d(LengthKt.getFeet(9.801), LengthKt.getFeet(20.523), Rotation2dKt.getDegree(120)));

		List<Pose2d> p_toLoadingStation = Arrays.asList(
				new Pose2d(LengthKt.getFeet(9.801), LengthKt.getFeet(20.523), Rotation2dKt.getDegree(120)),
				new Pose2d(LengthKt.getFeet(3.986), LengthKt.getFeet(24.816), Rotation2dKt.getDegree(180)));

		if (!isLeft) {
			// p_fallOffHab = Util.reflectTrajectory(p_fallOffHab);
			p_toLeft1 = Util.reflectTrajectory(p_toLeft1);
			p_halfWayToLoadingStationL = Util.reflectTrajectory(p_halfWayToLoadingStationL);
			p_toLoadingStation = Util.reflectTrajectory(p_toLoadingStation);
		}

		var t_farSideRocket = Trajectories.generateTrajectory(p_toLoadingStation, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(6)), kDefaultAcceleration, true, true);

		var t_halfWayToLoadingStationL = Trajectories.generateTrajectory(p_halfWayToLoadingStationL, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(6)), kDefaultAcceleration, false, true);

		var t_toLoadingStationL = Trajectories.generateTrajectory(p_toLoadingStation, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(6)), kDefaultAcceleration, false, true);

		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_fallOffHab, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // fall off the hab
		addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_farSideRocket, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // keep going over to the far side of the rocket
		addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		addSequential(new FollowVisionTargetTheSecond(3.8));
		addSequential(new RunIntake(-1, 0, 1));

		addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_halfWayToLoadingStationL, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // nyoom off to the side
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_toLoadingStationL, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); // go to the loading station
		// addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		addSequential(new FollowVisionTargetTheSecond(4.5));
		addSequential(new PIDDriveDistance(0.5, 4, /* timeout */ 0.5));
		addSequential(new RunIntake(1, 0, 1));
		addSequential(new PIDDriveDistance(-3, 12, /* timeout */ 1));

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
