/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.lib.motion.Util;

public class SetPoseFromVisionTarget extends InstantCommand {
	Pose2d goalPose;

	/**
	 * Eventually (tm) set the pose of the robot given the Pose2d of the vision target. TODO make this actually work
	 * @param goalPose the Pose2d of the vision target in the field reference frame
	 * 
	 * @author Matthew Morley
	 */
	public SetPoseFromVisionTarget(Pose2d goalPose) {
		this.goalPose = goalPose;
	}

	// Called just before this Command runs the first time
	@Override
	protected void execute() {
		double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});

		Translation2d mTranToGoal = new Translation2d(LengthKt.getInch(camtran[2]), LengthKt.getInch(camtran[0]));
		Rotation2d mRotToGoal = Rotation2dKt.getDegree(camtran[4]);
		Pose2d mPoseToGoal = new Pose2d(mTranToGoal, mRotToGoal);

		System.out.println(Util.toString(mPoseToGoal));

	}

}
