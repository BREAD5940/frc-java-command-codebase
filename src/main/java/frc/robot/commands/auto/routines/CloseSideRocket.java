package frc.robot.commands.auto.routines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

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
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.lib.OLDParallelRaceGroup;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;
import frc.robot.xboxmap;

/**
 * 2-hatch 1-cargo auto
 */
public class CloseSideRocket extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	// public CloseSideRocket(char arg1, char arg2) {
	// this();
	// }

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine. 
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public CloseSideRocket(char side) {

		boolean isLeft = side == 'L' || side == 'l';

		final Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
		final Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));

		final Velocity<Length> kDefaultVelocityLow = VelocityKt.getVelocity(LengthKt.getFeet(5));
		final Velocity<Length> kDefaultVelocityHigh = VelocityKt.getVelocity(LengthKt.getFeet(9));

		final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(6));

		var rocketC = Arrays.asList(
				new Pose2d(LengthKt.getFeet(5.331),
						LengthKt.getFeet(17.707),
						Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(9.039),
						LengthKt.getFeet(19.756),
						Rotation2dKt.getDegree(44.313))
		// new Pose2d(LengthKt.getFeet(15.712),
		// 	LengthKt.getFeet(24.817),
		// 	Rotation2dKt.getDegree(30))
		);

		if (!isLeft)
			rocketC = Util.reflectTrajectory(rocketC);

		var p_rocketC = Trajectories.generateTrajectory(
				rocketC,
				Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var rocketCPart2 = Arrays.asList(
				// new Pose2d(LengthKt.getFeet(5.331),
				// 	LengthKt.getFeet(17.707),
				// 	Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(9.039),
						LengthKt.getFeet(19.756),
						Rotation2dKt.getDegree(44.313)),
				new Pose2d(LengthKt.getFeet(14.707),
						LengthKt.getFeet(24.407),
						Rotation2dKt.getDegree(30)));

		if (!isLeft)
			rocketCPart2 = Util.reflectTrajectory(rocketCPart2);

		var p_rocketCPart2 = Trajectories.generateTrajectory(
				rocketCPart2,
				Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		// if (!isLeft) {

		// public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
		// List<? extends TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed, boolean optomizeSplines) {

		// var t_hab = Trajectories.generateTrajectory(p_hab, Trajectories.kLowGearConstraints, kDefaultStartVelocity,
		// 		VelocityKt.getVelocity(LengthKt.getFeet(/*7*/ 4)), VelocityKt.getVelocity(LengthKt.getFeet(/*7*/ 6)), kDefaultAcceleration, false, true);

		// var t_toPlaceHatch = Trajectories.generateTrajectory(p_toHatchPlace, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(/*7*/ 4)),
		// 		kDefaultEndVelocity, VelocityKt.getVelocity(LengthKt.getFeet(/*7*/ 5)), kDefaultAcceleration, false, true);

		// var t_halfWayToLoadingStationL = Trajectories.generateTrajectory(p_halfWayToLoadingStationL, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(0)),
		// 		VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(6)), kDefaultAcceleration, true, true);

		// var t_toLoadingStation = Trajectories.generateTrajectory(p_toLoadingStation, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(0)),
		// 		VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(6)), kDefaultAcceleration, false, true);

		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(p_rocketC, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // keep going over to the far side of the rocket
		addParallel(new JankyGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(p_rocketCPart2, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); // keep going over to the far side of the rocket

		Supplier<Boolean> checker = (() -> (Boolean.valueOf(Robot.m_oi.getPrimary().getRawButton(xboxmap.Buttons.A_BUTTON))));

		addSequential(new OLDParallelRaceGroup(checker, new TeleopCommands()));

		// addSequential(new DrivePower(-0.4, 0.5));

		addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));

		// addSequential(new PlaceHatch());

		// addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		// CommandGroup waitForABit = new CommandGroup();
		// waitForABit.addSequential(new WaitCommand("yes", 4));
		// waitForABit.addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		// addParallel(waitForABit);

		// addSequential(new WaitCommand(0.2));

		// addSequential(new FollowVisionTargetTheSecond(3.5));
		// addSequential(new DriveDistanceTheThird(LengthKt.getInch(6), false));
		// addSequential(new RunIntake(-1, 0, 1));

		// addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_halfWayToLoadingStationL, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); // nyoom off to the side
		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_toLoadingStation, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); // go to the loading station
		// // addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		// addSequential(new FollowVisionTargetTheSecond(4.5));
		// addSequential(new PIDDriveDistance(0.5, 4, /* timeout */ 0.5));
		// addSequential(new RunIntake(1, 0, 1));
		// addSequential(new PIDDriveDistance(-5, 12, /* timeout */ 1));

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
