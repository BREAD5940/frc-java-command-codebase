/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

public class DrivePowerToVisionTarget extends TimedCommand {

	double exitArea;
	boolean hasTarget, hadTarget;
	double steerK = 0.12;
	double foreCommand, turnCommand;

	double power;

	// InterpolatableLut mDeployedLookupTable, mStowedLookupTable;

	/**
	 * Line up left/right and allow the driver to drive forward/back.
	 * @param areaToExitAt when to exit the command (i.e. can't see the vision target anymore)
	 */
	public DrivePowerToVisionTarget(double power, double time) {
		super(time);

		requires(DriveTrain.getInstance());

		this.power = power;

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
		// double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
		double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

		tx = tx + (Robot.m_oi.getTurnAxis() * 10); // TODO tune, to offset allignment

		// InterpolatableLut lut;

		// if (SuperStructure.getInstance().getCurrentState().getElbowAngle().getDegree() > -30) {
		// deployed
		// lut = mDeployedLookupTable;
		// } else {
		// lut = mStowedLookupTable;
		// }

		// tx = tx + lut.interpolate(ts).doubleValue(); // Use the LUT to interpolate

		if (tv < 1.0) {
			hasTarget = false;
			turnCommand = 0.0;
			DriveTrain.getInstance().arcadeDrive(Robot.m_oi.getForwardAxis(), Robot.m_oi.getTurnAxis());
			return;
		} else {
			hasTarget = true;
			hadTarget = true;
		}

		// Rotation2d oldTargetYaw = Rotation2dKt.getDegree(tx).times(-1); // TODO check signs!
		// Time measurementTime = TimeUnitsKt.getSecond(Timer.getFPGATimestamp()).minus(LimeLight.getInstance().getPipelineLatency());
		// Rotation2d robotYawAtMeasurementTime = DriveTrain.getInstance().getLocalization().get(measurementTime).getRotation();
		// Rotation2d measuredTargetGlobalYaw = robotYawAtMeasurementTime.plus(oldTargetYaw); // TODO check sign!
		// Rotation2d currentRobotYaw = DriveTrain.getInstance().getLocalization().getRobotPosition().getRotation();
		// Rotation2d realOffsetNow = measuredTargetGlobalYaw.minus(currentRobotYaw); // TODO check sign

		// // store this value again
		// tx = realOffsetNow.getDegree();

		double steer_cmd;// =  (isSmallAngle) ? tx * steerK : 0.5 * Math.signum(tx); 

		boolean isHighGear = (Robot.getDrivetrainGear() == Gear.HIGH);

		// System.out.println("Is high gear? " + isHighGear);

		// System.out.println("IS HIGH GEAR??? " + isHighGear);
		double closeK;//, nearK, farK;

		double threshold = 8;

		if (isHighGear) {
			closeK = 0.1;
			// nearK = 0.05;
			// farK = 0.4;
		} else {
			closeK = 0.1;
			// nearK = 0.06;
			// farK = 0.55;
		}

		// if (Math.abs(tx) < 5) {
		steer_cmd = tx * closeK;
		steer_cmd = Util.limit(steer_cmd, 0.4);
		// // } else if (Math.abs(tx) < 10) {
		// // 	steer_cmd = tx * nearK;
		// } else {
		// 	steer_cmd = farK * Math.signum(tx);
		// }

		double fore_cmd = power;

		if (hasTarget) {
			System.out.println("CommandP: " + fore_cmd + "," + steer_cmd);
			DriveTrain.getInstance().arcadeDrive(fore_cmd, steer_cmd);
		} else {
			DriveTrain.getInstance().stop();
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	// @Override
	// protected boolean isFinished() {
	// var ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
	// return /*(Math.abs(exitArea - ta) < 0.3) || (hadTarget && !hasTarget)*/ isTimedOut();
	// }

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
