/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class FollowVisionTargetTheSecond extends Command {

	double targetArea;
	boolean mHadTarget = false;

	public FollowVisionTargetTheSecond(double targetArea) {
		requires(DriveTrain.getInstance());
		this.targetArea = targetArea;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// just in case turn on LEDs
		LimeLight.getInstance().turnOnLED();

		m_isDone = false;
		// SuperStructure.getElevator().getMaster().configPeakOutputForward(0);
		// SuperStructure.getElevator().getMaster().configPeakOutputReverse(0);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Update_Limelight_Tracking();
		if (m_LimelightHasValidTarget) {
			DriveTrain.getInstance().arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand);
		} else {
			DriveTrain.getInstance().stop();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return m_isDone || (mHadTarget && !m_LimelightHasValidTarget);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}

	private boolean m_LimelightHasValidTarget = false;
	private double m_LimelightDriveCommand = 0.0;
	private double m_LimelightSteerCommand = 0.0;
	private boolean m_isDone = false;

	/**
	 * This function implements a simple method of generating driving and steering commands
	 * based on the tracking data from a limelight camera.
	 */
	public void Update_Limelight_Tracking() {
		// These numbers must be tuned for your Robot!  Be careful!
		double STEER_K = 0.04;                    // how hard to turn toward the target
		double DRIVE_K = 0.26 * 1.6 * 1.3 / 1.4;                    // how hard to drive fwd toward the target
		double DESIRED_TARGET_AREA = this.targetArea;         // Area of the target when the robot reaches the wall
		double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

		double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
		double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

		if (Math.abs(DESIRED_TARGET_AREA - ta) < 0.3) {
			System.out.println("WE DONE BOIS");
			m_isDone = true;
		}

		System.out.println("DESIRED_TARGET_AREA - ta: " + (DESIRED_TARGET_AREA - ta));

		if (tv < 1.0) {
			m_LimelightHasValidTarget = false;
			m_LimelightDriveCommand = 0.0;
			m_LimelightSteerCommand = 0.0;
			return;
		}

		m_LimelightHasValidTarget = true;

		mHadTarget = true;

		// Start with proportional steering
		double steer_cmd = tx * STEER_K;
		m_LimelightSteerCommand = steer_cmd;

		// try to drive forward until the target area reaches our desired area
		double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
		drive_cmd = drive_cmd + 1d / 12d * Math.signum(drive_cmd);

		// don't let the robot drive too fast into the goal
		if (drive_cmd > MAX_DRIVE) {
			drive_cmd = MAX_DRIVE;
		}
		m_LimelightDriveCommand = drive_cmd;
	}
}
