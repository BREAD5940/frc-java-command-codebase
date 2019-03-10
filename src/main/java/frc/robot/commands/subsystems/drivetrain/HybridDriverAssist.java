/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

public class HybridDriverAssist extends Command {

	double exitArea;
	boolean hasTarget, hadTarget;
	double steerK = 0.12;
	double foreCommand, turnCommand;

	/**
	 * Line up left/right and allow the driver to drive forward/back.
	 * @param areaToExitAt when to exit the command (i.e. can't see the vision target anymore)
	 */
	public HybridDriverAssist(double areaToExitAt) {
		this.exitArea = areaToExitAt;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(DriveTrain.getInstance());
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
		double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

		if (tv < 1.0) {
			hasTarget = false;
			turnCommand = 0.0;
			return;
		} else {
			hasTarget = true;
			hadTarget = true;
		}

		// var isSmallAngle = (Math.abs(tx) < 10);

		double steer_cmd;// =  (isSmallAngle) ? tx * steerK : 0.5 * Math.signum(tx); 

		boolean isHighGear = (Robot.getDrivetrainGear() == Gear.HIGH);

		// System.out.println("IS HIGH GEAR??? " + isHighGear);
		double closeK, nearK, farK;

		if (isHighGear) {
			closeK = 0.1;
			nearK = 0.07;
			farK = 0.4;
		} else {
			closeK = 0.1;
			nearK = 0.06;
			farK = 0.55;
		}

		if (Math.abs(tx) < 5) {
			steer_cmd = tx * closeK;
		} else if (Math.abs(tx) < 10) {
			steer_cmd = tx * nearK;
		} else {
			steer_cmd = farK * Math.signum(tx);
		}

		double fore_cmd = Robot.m_oi.getForwardAxis();

		if (hasTarget) {
			DriveTrain.getInstance().arcadeDrive(fore_cmd, steer_cmd);
		} else {
			DriveTrain.getInstance().stop();
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		var ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
		return (Math.abs(exitArea - ta) < 0.3) || (hadTarget && !hasTarget);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		DriveTrain.getInstance().stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
