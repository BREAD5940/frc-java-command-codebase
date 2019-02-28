/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import java.util.function.Consumer;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetPoseFromVisionTarget extends InstantCommand {
	Consumer<Pose2d> mRun;

	public SetPoseFromVisionTarget(Consumer<Pose2d> run) {
		this.mRun = run;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
		if (camtran.length < 5) {
			System.out.println("oof");
		} else {
			Length x = LengthKt.getInch(5);//camtran[0]);
			Length y = LengthKt.getInch(4);//camtran[1]);
			Rotation2d yaw = Rotation2dKt.getDegree(4);

			Pose2d newPose = new Pose2d(x, y, yaw);
			System.out.println("setting new pose");
			mRun.accept(newPose);
		}

	}
}
