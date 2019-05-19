/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import com.team254.lib.physics.DifferentialDrive.ChassisState;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;

public class HybridKinematicDriverAssist extends SendableCommandBase {

	double exitArea;
	boolean hasTarget, hadTarget;
	double steerK = 0.12;
	double foreCommand, turnCommand;

	/**
	 * Line up left/right and allow the driver to drive forward/back.
	 * 
	 * @param areaToExitAt when to exit the command (i.e. can't see the vision
	 *                     target anymore)
	 */
	public HybridKinematicDriverAssist() {
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		addRequirements(DriveTrain.getInstance());
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {}

	double lastTx = 0;
	double lastGyroAngle = 0;
	double lastDistanceToTurn = 0;
	final boolean doGyroCompensation = true; // FIXME make me off if you wanna kill it

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
		double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

		var comped_tx = tx - (Robot.m_oi.getTurnAxis() * 8); // TODO tune, to offset allignment
		var mGyroAngle = DriveTrain.getInstance().getGyro();
		var distToTurn = comped_tx;

		// if (tx == lastTx && doGyroCompensation) { // if same use gyro
		// distToTurn = lastTx + (lastTx - tx) - (Robot.m_oi.getTurnAxis() * 15);
		// }

		if (tv < 1.0) {
			hasTarget = false;
			turnCommand = 0.0;
			DriveTrain.getInstance().arcadeDrive(Robot.m_oi.getForwardAxis(), Robot.m_oi.getTurnAxis());
			return;
		} else {
			hasTarget = true;
			hadTarget = true;
		}

		double fore_cmd = Robot.m_oi.getForwardAxis();

		final double foreMaxSpeed = Util.toMeters(6); // meters
		final double turnMaxSpeed = 4; // ft

		final double turn_kp = 0.04;
		double turn_cmd = Util.toMeters(distToTurn * turn_kp * turnMaxSpeed * -1);

		ChassisState mCalcedState = new ChassisState(fore_cmd * foreMaxSpeed, turn_cmd);
		// DriveTrain.getInstance().getDifferentialDrive().solveInverseKinematics(mCalcedState);

		System.out.println("desired chassis state: " + mCalcedState.getLinear() + "," + mCalcedState.getAngular());

		if (hasTarget) {
			DriveTrain.getInstance().setOutputFromKinematics(mCalcedState);
		} else {
			DriveTrain.getInstance().coast();
		}

		lastTx = tx;
		lastGyroAngle = mGyroAngle;
		lastDistanceToTurn = distToTurn;

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return /*(Math.abs(exitArea - ta) < 0.3) || (hadTarget && !hasTarget)*/ false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		DriveTrain.getInstance().stop();
	}

}
