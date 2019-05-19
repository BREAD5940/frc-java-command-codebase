/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.LimeLight;

public class FollowVisionTargetTheSecond extends SendableCommandBase {

	double targetArea;
	boolean mHadTarget = false;
	double offset = 0;

	public FollowVisionTargetTheSecond(double targetArea) {
		addRequirements(DriveTrain.getInstance());
		this.targetArea = targetArea;
	}

	public FollowVisionTargetTheSecond(double targetArea, double angleOffset) {
		addRequirements(DriveTrain.getInstance());
		this.targetArea = targetArea;
		this.offset = angleOffset;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		// just in case turn on LEDs
		LimeLight.getInstance().turnOnLED();

		m_isDone = false;
		// SuperStructure.getElevator().getMaster().configPeakOutputForward(0);
		// SuperStructure.getElevator().getMaster().configPeakOutputReverse(0);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		Update_Limelight_Tracking();
		if (m_LimelightHasValidTarget) {
			// System.out.println(m_LimelightDriveCommand);
			DriveTrain.getInstance().arcadeDrive(m_LimelightDriveCommand,
					m_LimelightSteerCommand);
		} else {
			DriveTrain.getInstance().stop();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return m_isDone || (mHadTarget && !m_LimelightHasValidTarget);
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {}

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
		double STEER_K = 0.4;                    // how hard to turn toward the target
		double DRIVE_K = 0.26 * 1.6 * 1.3 / 1;                    // how hard to drive fwd toward the target
		double DESIRED_TARGET_AREA = this.targetArea;         // Area of the target when the robot reaches the wall
		double MAX_DRIVE = 0.45;                   // Simple speed limit so we don't drive too fast

		double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv")
				.getDouble(0);
		double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
				.getDouble(0) - offset;
		double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty")
				.getDouble(0);
		double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta")
				.getDouble(0);

		if (ta < 1)
			MAX_DRIVE = .7; // TOOD tune

		if (Math.abs(DESIRED_TARGET_AREA - ta) < 0.3) {
			System.out.println("WE DONE BOIS");
			m_isDone = true;
		}

		// System.out.println("DESIRED_TARGET_AREA - ta: " + (DESIRED_TARGET_AREA - ta));

		if (tv < 1.0) {
			m_LimelightHasValidTarget = false;
			m_LimelightDriveCommand = 0.0;
			m_LimelightSteerCommand = 0.0;
			return;
		}

		m_LimelightHasValidTarget = true;

		mHadTarget = true;

		// Start with proportional steering
		// double steer_cmd = tx * STEER_K;

		double closeK, nearK, farK;
		double steer_cmd;
		boolean isHighGear = (Robot.getDrivetrainGear() == Gear.HIGH);

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
