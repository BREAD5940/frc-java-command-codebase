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

import frc.robot.Robot;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.actions.WaitForOI;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.drivetrain.ArcadeDrive;
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
public class FarSideRocketHybrid extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	// public FarSideRocket(char arg1, char arg2) {
	// this();
	// }

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine.
	 * 
	 * @param side     to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public FarSideRocketHybrid(char side) {

		boolean isLeft = (side == 'L' || side == 'l');

		final Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
		final Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));

		final Velocity<Length> kDefaultVelocityLow = VelocityKt.getVelocity(LengthKt.getFeet(4));
		final Velocity<Length> kDefaultVelocityHigh = VelocityKt.getVelocity(LengthKt.getFeet(9));

		final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(6));

		List<Pose2d> p_fallOffHab = Arrays.asList(
				new Pose2d(
						LengthKt.getFeet(5.25),
						LengthKt.getFeet(17.65),
						Rotation2dKt.getDegree(180)),
				new Pose2d(
						LengthKt.getFeet(11),
						LengthKt.getFeet(17.65),
						Rotation2dKt.getDegree(180)));

		List<Pose2d> p_farSideRocketL = Arrays.asList(
				new Pose2d(
						LengthKt.getFeet(11),
						LengthKt.getFeet(17.65),
						Rotation2dKt.getDegree(180)),

				new Pose2d(
						LengthKt.getFeet(19.787),
						LengthKt.getFeet(21.655),
						Rotation2dKt.getDegree(-140)),

				new Pose2d(
						LengthKt.getFeet(24.118),
						LengthKt.getFeet(23.658),
						Rotation2dKt.getDegree(150.0)));

		if (!isLeft) {
			p_fallOffHab = Util.reflectTrajectory(p_fallOffHab);
			p_farSideRocketL = Util.reflectTrajectory(p_farSideRocketL);

		}

		// public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
		// List<? extends TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed, boolean optomizeSplines) {

		var t_fallOffHab = Trajectories.generateTrajectory(p_fallOffHab, Trajectories.kLowGearConstraints, kDefaultStartVelocity,
				VelocityKt.getVelocity(LengthKt.getFeet(4)), kDefaultVelocityLow, kDefaultAcceleration, true, true);

		var t_farSideRocketL = Trajectories.generateTrajectory(p_farSideRocketL, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(4)),
				VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(7)), kDefaultAcceleration, true, true);

		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_fallOffHab, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // fall off the hab
		addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_farSideRocketL, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // keep going over to the far side of the rocket
		addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		addSequential(new FollowVisionTargetTheSecond(3.5));
		
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
