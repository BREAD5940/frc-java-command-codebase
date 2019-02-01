package frc.robot.lib.motion;

import frc.math.Pose2d;
import frc.math.Rotation2d;
import frc.robot.lib.motion.purepursuit.Point;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import frc.robot.Robot;

public class Odometer /* extends LogBase */ {
	private double accumulativeDistance;
	private double x;
	private double y;
	private double theta;

	private double prevLeftEncoderValue;
	private double prevRightEncoderValue;
	private static Odometer instance;

	private boolean flipped = false;

	public static Odometer getInstance() {
		if (instance == null)
			instance = new Odometer();
		return instance;
	}

	public Odometer() {
		// setLogSenderName("Odometer");
		reset();
		// log("Started");
		// System.out.println("Odometer started");

	}

	public void setOdometryForPathfinder(Trajectory trajectory){
		setX(trajectory.get(0).x);
		setY(trajectory.get(0).y);
		// setTheta(trajectory.get(0).heading);
	}

	public void update(double leftEncoder, double rightEncoder, double gyroAngle) {
		this.theta = Math.toRadians(Pathfinder.boundHalfDegrees(Robot.drivetrain.getGyro()) );

		gyroAngle = (flipped) ? gyroAngle - 270 : gyroAngle + 90;

		gyroAngle = Math.toRadians(gyroAngle);

		gyroAngle = Math.PI - gyroAngle;

		double deltaLeftEncoder = leftEncoder - prevLeftEncoderValue;
		double deltaRightEncoder = rightEncoder - prevRightEncoderValue;
		double distance = (deltaLeftEncoder + deltaRightEncoder) / 2;

		x += distance * Math.cos(gyroAngle);
		y += distance * Math.sin(gyroAngle);

		// x -= distance * Math.cos(gyroAngle);
		// y += distance * Math.sin(gyroAngle);

		accumulativeDistance += Math.abs(distance);
		prevLeftEncoderValue = leftEncoder;
		prevRightEncoderValue = rightEncoder;
	}

	public void reset() {
		accumulativeDistance = x = y = prevLeftEncoderValue = prevRightEncoderValue = 0;
		// log("reset");
	}

	public double getAccumulativeDistance() {
		return accumulativeDistance;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getTheta() {
		return theta % (2 * Math.PI);
	}

	public synchronized void setTheta(double theta){
		this.theta = theta;
	}

	@Deprecated
	public void setX(double x) {
		this.x = x;
	}

	@Deprecated
	public void setY(double y) {
		this.y = y;
	}

	public void setPose(Pose2d pose) {
		setX(pose.getTranslation().x());
		setY(pose.getTranslation().y());
		setTheta(pose.getRotation().getDegrees());
	}

	public void reverseXY(boolean reversed) {
		this.flipped = reversed;
	}

	public Pose2d getPose() {
				return new Pose2d(x, y, Rotation2d.fromDegrees(getTheta()));
	}

	public Point getPoint() {
			return new Point(x, y);
	}

	@Override
	public String toString() {
		return "Odometer: accumulativeDistance=" + accumulativeDistance + ", x=" + x + ", y=" + y;
	}

}
