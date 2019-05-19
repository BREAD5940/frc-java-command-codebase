package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.lib.obj.HalfBakedRotatingSRX;
import frc.robot.lib.obj.RoundRotation2d;
import frc.team254.Trajectory;

public class RotatingVelocityTrajectoryFollower {

	HalfBakedRotatingSRX mMotor;

	Trajectory profile;

	private int index = 0;

	double kP, kV, kA;

	public RotatingVelocityTrajectoryFollower(HalfBakedRotatingSRX mMotor,
			Trajectory profile, double kP, double kV,
			double kA) {
		this.mMotor = mMotor;
		this.profile = profile;
		this.kV = kV;
		this.kA = kA;
		this.kP = kP;
	}

	public mPeriodicIO run() {
		var seg = profile.getSegment(index);
		index += 1;
		var mKv = seg.vel * kV;
		var mKa = seg.acc * kA;
		var pos = RoundRotation2d.getDegree(seg.pos);
		var mError = pos.minus(mMotor.getRotation2d()).absoluteValueOf();
		var mKp = mError.getDegree() * kP;
		var feedForward = mKp + mKv + mKa;
		var feedBack = RoundRotation2d.getDegree(seg.vel);

		return new mPeriodicIO(feedBack, feedForward, ControlMode.Velocity);
	}

	public boolean isDone() {
		return false;
	}

	public class mPeriodicIO {
		RoundRotation2d fb;
		double ff;
		ControlMode mode;

		public mPeriodicIO(RoundRotation2d feedBack, double feedForward,
				ControlMode mode) {
			this.ff = feedForward;
			this.fb = feedBack;
			this.mode = mode;
		}
	}

}
