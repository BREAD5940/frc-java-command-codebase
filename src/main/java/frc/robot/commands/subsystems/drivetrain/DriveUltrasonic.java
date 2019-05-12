///*----------------------------------------------------------------------------*/
///* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
///* Open Source Software - may be modified and shared by FRC teams. The code   */
///* must be accompanied by the FIRST BSD license file in the root directory of */
///* the project.                                                               */
///*----------------------------------------------------------------------------*/
//
//package frc.robot.commands.subsystems.drivetrain;
//
//import org.ghrobotics.lib.mathematics.units.LengthKt;
//import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
//
//import org.team5940.pantry.exparimental.command.SendableCommandBase;
//import frc.robot.Robot;
//import frc.robot.lib.TerriblePID;
//import frc.robot.lib.obj.DriveSignal;
//import frc.robot.subsystems.DifferentialUltrasonicSensor;
//import frc.robot.subsystems.DifferentialUltrasonicSensor.RangeMode;
//
//public class DriveUltrasonic extends SendableCommandBase {
//
//	double targetAngle = 0;
//
//	double maxSpeed;
//	double acceleration;
//	double targetDistance;
//
//	double angle, distance;
//
//	double tolerence = 0.1; // units are in feet
//
//	private double timeout = 15;
//
//	private boolean gotReading;
//
//	private double v_i;
//
//	private double[] data;
//
//	public DriveUltrasonic() {
//		this(0, 5, 10, 2, 15);
//	}
//
//	double turnOutput;
//
//	double forwardOutput;
//
//	private boolean deceleratePhase;
//
//	DriveSignal mSignal;
//
//	private TerriblePID turnPid = new TerriblePID(0.015, 0, 0.1, 0, -0.5, 0.5, 0, 0, 0.1, null, null);
//
//	/**
//	 * Drive to a specified distance at a target speed, while pushing the heading to be at a specified angle from the wall.
//	 * Distance is calculated based on the minimum value of the two sensors
//	 * @param angle in degrees
//	 * @param maxSpeed in ft/sec
//	 * @param acceleration in ft/sec/sec
//	 * @param targetDistance in feet
//	 */
//	public DriveUltrasonic(double angle, double maxSpeed, double acceleration, double targetDistance, double timeout) {
//		addRequirements(Robot.drivetrain);
//		this.maxSpeed = maxSpeed;
//		this.acceleration = acceleration;
//		this.targetDistance = targetDistance;
//		this.targetAngle = angle;
//		this.timeout = timeout;
//		turnPid.setSetpoint(angle);
//	}
//
//	// Called just before this Command runs the first time
//	@Override
//	public void initialize() {
//		DifferentialUltrasonicSensor.getInstance().setMode(RangeMode.FEET);
//	}
//
//	// Called repeatedly when this Command is scheduled to run
//	@Override
//	public void execute() {
//		data = DifferentialUltrasonicSensor.getInstance().getRangesAsDouble();
//		distance = Math.min(data[0], data[1]);
//		angle = DifferentialUltrasonicSensor.getInstance().getAngleOffset();
//
//		if (distance != 0) {
//			gotReading = true;
//		}
//
//		// from the angle and distance, we can drive a P loop on distance
//		// and a PID loop on angle
//		turnOutput = turnPid.update(Robot.drivetrain.getGyro());
//
//		// for the forward velocity, we try to drive a trapezoidal velocity
//		// given the kinematic equations we find that the distance required to accelerate
//		// to zero velocity is given by d = v_i^2 / (2 * acceleration), so we drive the max
//		// velocity uless the distance is below the distance needed to stop times
//		// a fudge factor.
//
//		// If we are just now reaching the deceleration phase
//		if (deceleratePhase == false && distance < 1.1 * Math.pow((Robot.drivetrain.getLeft().getFeetPerSecond() + Robot.drivetrain.getRight().getFeetPerSecond()) / 2/* TODO make this use robot speed*/ , 2) / (2 * acceleration)) {
//			deceleratePhase = true;
//			v_i = (Robot.drivetrain.getLeft().getFeetPerSecond() + Robot.drivetrain.getRight().getFeetPerSecond()) / 2;
//		}
//
//		// use a conditional operator to decide the target speed
//		forwardOutput = (deceleratePhase) ? v_i - acceleration * distance : maxSpeed;
//
//		mSignal = new DriveSignal(VelocityKt.getVelocity(LengthKt.getFeet(forwardOutput + turnOutput)), VelocityKt.getVelocity(LengthKt.getFeet(forwardOutput - turnOutput)));
//
//		Robot.drivetrain.setClosedLoop(mSignal);// (forwardOutput + turnOutput, forwardOutput - turnOutput);
//	}
//
//	// Make this return true when this Command no longer needs to run execute()
//	@Override
//	public boolean isFinished() {
//		if (distance != 0 || gotReading) { // if we have a reading
//			return Math.abs(distance) < tolerence || isTimedOut();
//		} else {
//			return isTimedOut();
//		}
//	}
//
//	// Called once after isFinished returns true
//	@Override
//	public void end(boolean interrupted) {}
//
//	// Called when another command which requires one or more of the same
//	// subsystems is scheduled to run
//	@Override
//	protected void interrupted() {}
//}
