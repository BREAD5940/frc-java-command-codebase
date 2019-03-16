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
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
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

		var p_toFarSide = Arrays.asList(
			new Pose2d(LengthKt.getFeet(5.227),
				LengthKt.getFeet(17.7),
				Rotation2dKt.getDegree(180)),
			new Pose2d(LengthKt.getFeet(9.975),
				LengthKt.getFeet(17.7),
				Rotation2dKt.getDegree(180)),
			new Pose2d(LengthKt.getFeet(19.837),
				LengthKt.getFeet(22.322),
				Rotation2dKt.getDegree(-139.374)),
			new Pose2d(LengthKt.getFeet(23.46),
				LengthKt.getFeet(23.979),
				Rotation2dKt.getDegree(150))
		);

		if (!isLeft) {
			p_toFarSide = Util.reflectTrajectory(p_toFarSide);
			// p_farSideRocketL = Util.reflectTrajectory(p_farSideRocketL);

		}

		// public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
		// List<? extends TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed, boolean optomizeSplines) {

		var t_toFarSide = Trajectories.generateTrajectory(p_toFarSide, Trajectories.kLowGearConstraints, kDefaultStartVelocity,
				VelocityKt.getVelocity(LengthKt.getFeet(6)), kDefaultVelocityLow, kDefaultAcceleration, true, true);


		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_toFarSide, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // fall off the hab
		addSequential(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		// addSequential(new FollowVisionTargetTheSecond(3.5));

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
