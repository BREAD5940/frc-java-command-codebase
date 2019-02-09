/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.TerriblePID;
import frc.robot.lib.TerriblePID.FeedForwardBehavior;
import frc.robot.lib.TerriblePID.FeedForwardMode;

/**
 * Literally just pivot in place by a desired amount
 */
public class TurnInPlace extends Command {

	double starting_angle;
	double target_angle_relative;
	double target_angle;
	double target_angle_absolute;
	boolean isAbsolute = false;
	double output;
	double max_turn_speed;
	double raw_left;
	double raw_right;

	// TerriblePID turnPID = new TerriblePID(RobotConfig.auto.TurnInPlace.kp, RobotConfig.auto.TurnInPlace.ki,
	//   RobotConfig.auto.TurnInPlace.min_turn_speed,
	//   RobotConfig.auto.TurnInPlace.max_turn_speed,
	//   RobotConfig.auto.TurnInPlace.integral_zone,
	//   RobotConfig.auto.TurnInPlace.max_integral);

	// public TerriblePID(double kp, double ki, double minOutput, double maxOutput, double integralZone, double maxIntegralAccum, double kf, FeedForwardMode feedforwardmode, FeedForwardBehavior feedforwardbehavior, double unitsPerQuarterWave) {
	TerriblePID turnPID = new TerriblePID(RobotConfig.auto.turnInPlace.kp, RobotConfig.auto.turnInPlace.ki, 0d, RobotConfig.auto.turnInPlace.kf,
			RobotConfig.auto.turnInPlace.min_turn_speed,
			RobotConfig.auto.turnInPlace.max_turn_speed,
			RobotConfig.auto.turnInPlace.integral_zone,
			RobotConfig.auto.turnInPlace.max_integral,
			100000,
			FeedForwardMode.LINEAR,
			FeedForwardBehavior.NORMAL);

	/**
	 * Turn a specified number of degrees in the default auto gear.
	 * This constructor will default to taking the angle relative to
	 * the robot's angle when the command is initialized, not the
	 * absolute angle. If you want to specify, use a bool as the second
	 * argument to specify if the angle should be interpreted as absolute
	 * or not.
	 * @param target_angle angle the robot should turn to in degrees
	 */
	public TurnInPlace(double target_angle) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
		this.target_angle = target_angle;
	}

	/**
	 * Turn a specified number of degrees in the default auto gear.
	 * The angle passed is an absolute angle relative to the
	 * angle upon autonomous init.
	 * @param target_angle angle robot should turn to in degrees
	 * @param isAbsolute true if angle is absolute relative to auto init
	 */
	public TurnInPlace(double target_angle, boolean isAbsolute) {
		this.isAbsolute = isAbsolute;
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
		this.target_angle = target_angle;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		starting_angle = Robot.drivetrain.getGyro();

		// If the angle is relative (which it should not be), setup target angle.
		// Otherwise the angle is absolute (relative to auto init) so we don't care.
		if (!(isAbsolute)) { // if isAbsolute is false, and we want a relative angle
			target_angle = target_angle + starting_angle;
		}

		turnPID.setSetpoint(target_angle);
		System.out.println("Turn in place init'ed!");

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		output = turnPID.update(Robot.drivetrain.getGyro());
		// raw_left = EncoderLib.distanceToRaw(output, RobotConfig.driveTrain.left_wheel_effective_diameter, RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);
		// raw_right = (-1) * EncoderLib.distanceToRaw(output, RobotConfig.driveTrain.right_wheel_effective_diameter, RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);
		// Robot.drivetrain.setSpeeds(raw_left, raw_right);
		Robot.drivetrain.setClosedLoop(null, null);
		System.out.println(String.format("Turn in place execute! Gyro output: %s,Output: %s, Raw left: %s Raw right %s", Robot.drivetrain.getGyro(), output, raw_left, raw_right));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// if ( (Math.abs(Robot.gyro.getRate() ) < RobotConfig.turn_auto_angular_velocity_tolerence)
		//   && (Math.abs(Robot.drivetrain.getGyro()) < RobotConfig.turn_auto_angle_tolerence)) {
		//     return true;
		//   } else { return false; }

		// TODO so this is how a return works
		return ((Math.abs(Robot.drivetrain.gyro.getRate()) < RobotConfig.auto.tolerences.angular_velocity_tolerence)
				&& (Math.abs(Robot.drivetrain.getGyro() - target_angle) < RobotConfig.auto.tolerences.angle_tolerence));

	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
