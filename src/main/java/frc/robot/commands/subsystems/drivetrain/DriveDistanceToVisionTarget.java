/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.LimeLight;

public class DriveDistanceToVisionTarget extends CommandGroup {
	double targetDistance, exitArea;

	// final Pose2d initialPose;
	Length kEndOffset, straightLength;

	/**
	 * Follow a straight trajectory to a vision target. This command gets the vision target pose on first init and will reset the robot odometry.
	 * 
	 * @author Matthew Morley
	 * 
	 * @param desired_end how far away to end at
	 * @param areaAtWhichToExit how far away to end this command at regardless of tracker state
	 */
	public DriveDistanceToVisionTarget(/*Pose2d currentPose, */Length desired_end, double areaAtWhichToExit) {
		// this.targetDistance = targetLimelightOffset;
		this.exitArea = areaAtWhichToExit;
		// this.initialPose = currentPose;
		this.kEndOffset = desired_end;
		// this.straightLength = driveStraightDistance;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(DriveTrain.getInstance());
	}

	boolean mCommandStarted = false;
	TimedTrajectory<Pose2dWithCurvature> trajectory;
	Command mFollowerCommand;

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		final Length kOffset = LengthKt.getInch(100); // so that the spline ends up in FalconDashboard instead of off the map. HOly shit the possibilty for bugs here is big. Let's not.

		double now = Timer.getFPGATimestamp();

		// get the current and target poses in addition to drive straight pose
		Pose2d mVisionTargetPose = LimeLight.getInstance().getPose(kOffset.getInch());

		// var poseAtMeasurement = DriveTrain.getInstance().getLocalization().get(TimeUnitsKt.getSecond(Timer.getFPGATimestamp() - LimeLight.getInstance().getPipelineLatency()));
		// var currentPose = DriveTrain.getInstance().getLocalization().getRobotPosition();

		// var deltaLen = currentPose.minus(poseAtMeasurement).getTranslation().getNorm();
		// var deltaAngle = currentPose.getRotation().minus(poseAtMeasurement.getRotation());
		// var deltaPose = new Pose2d(deltaLen, rotation)

		mVisionTargetPose = new Pose2d(mVisionTargetPose.getTranslation().getX(), kOffset, Rotation2dKt.getDegree(0));
		Pose2d end = new Pose2d(new Translation2d(kOffset.minus(kEndOffset), kOffset), Rotation2dKt.getDegree(0));

		System.out.println(String.format("Current pose: %s end poseL: %s", Util.toString(mVisionTargetPose), Util.toString(end)));

		// offset the end by the end offset to make a straight portion
		// Pose2d splineEnd = end.minus(new Pose2d(kOffset.plus(kEndOffset).minus(straightLength), LengthKt.getInch(0), Rotation2dKt.getDegree(0)));

		List<Pose2d> path;
		// // just go straight if we are too close or straight distance is zero
		// if (end.getTranslation().getX().epsilonEquals(splineEnd.getTranslation().getX()) || (mVisionTargetPose.getTranslation().getX().minus(kOffset).getInch() < straightLength.getInch())) {
		// 	path = Arrays.asList(mVisionTargetPose, end);
		// } else {
		// 	path = Arrays.asList(mVisionTargetPose, splineEnd, end);
		// }

		path = Arrays.asList(mVisionTargetPose, end);

		trajectory = Trajectories.generateTrajectory(path, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(2)), Trajectories.kDefaultAcceleration.div(2), false, true);
		mFollowerCommand = DriveTrain.getInstance().followTrajectoryWithGear(trajectory, TrajectoryTrackerMode.RAMSETE, Gear.HIGH, true);

		System.out.println("========== Vision spline generated in " + (Timer.getFPGATimestamp() - now) + " seconds! ==========");

		System.out.println("Pose2d of the target that we measured: " + Util.toString(mVisionTargetPose));

		// this.clearRequirements();
		mFollowerCommand.start();
		mCommandStarted = true;

		// addSequential(mFollowerCommand);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// if(mCommandStarted && mFollowerCommand.isCompleted()) requires(DriveTrain.getInstance());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (mCommandStarted && mFollowerCommand.isCompleted()) || (LimeLight.getInstance().getTargetArea() > exitArea);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		DriveTrain.getInstance().stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
