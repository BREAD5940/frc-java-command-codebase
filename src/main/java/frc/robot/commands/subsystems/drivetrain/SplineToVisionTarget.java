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

public class SplineToVisionTarget extends CommandGroup {
	double targetDistance, exitArea;

	// final Pose2d initialPose;
	Length kEndOffset, straightLength;

	/**
	 * Follow a spline shaped trajectory to a vision target. This command gets the vision target pose on first init and will reset the robot odometry.
	 * 
	 * @author Matthew Morley
	 * 
	 * @param currentPose the current pose of the robot (vision target centric fama)
	 * @param driveStraightDistance the distance to drive straight for
	 * @param dsired_end how far away from the vision target to exit at
	 * @param areaAtWhichToExit the area at which the command will exit, regarless of everything else
	 */
	public SplineToVisionTarget(/*Pose2d currentPose, */Length driveStraightDistance, Length desired_end, double areaAtWhichToExit) {
		// this.targetDistance = targetLimelightOffset;
		this.exitArea = areaAtWhichToExit;
		// this.initialPose = currentPose;
		this.kEndOffset = desired_end;
		this.straightLength = driveStraightDistance;
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
		Pose2d mVisionTargetPose = LimeLight.getInstance().getPose(kEndOffset.getInch(), kOffset.getInch());
		Pose2d end = new Pose2d(new Translation2d(kOffset.plus(kEndOffset), kOffset), Rotation2dKt.getDegree(0));
		// offset the end by the end offset to make a straight portion
		Pose2d splineEnd = end.minus(new Pose2d(kOffset.plus(kEndOffset).minus(straightLength), LengthKt.getInch(0), Rotation2dKt.getDegree(0)));

		List<Pose2d> path;
		// just go straight if we are too close or straight distance is zero
		if (end.getTranslation().getX().epsilonEquals(splineEnd.getTranslation().getX()) || (mVisionTargetPose.getTranslation().getX().minus(kOffset).getInch() < straightLength.getInch())) {
			path = Arrays.asList(mVisionTargetPose, end);
		} else {
			path = Arrays.asList(mVisionTargetPose, splineEnd, end);
		}

		trajectory = Trajectories.generateTrajectory(path, Trajectories.kLowGearConstraints, VelocityKt.getVelocity(LengthKt.getFeet(1)), VelocityKt.getVelocity(LengthKt.getFeet(2)), VelocityKt.getVelocity(LengthKt.getFeet(2)), Trajectories.kDefaultAcceleration.div(2), false, true);
		mFollowerCommand = DriveTrain.getInstance().followTrajectoryWithGear(trajectory, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true);

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
