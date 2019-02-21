package frc.robot.lib;

import frc.robot.lib.TerriblePID.FeedForwardBehavior;
import frc.robot.lib.TerriblePID.FeedForwardMode;

public class PIDSettings {

	public PIDSettings(double kp_, double ki_, double kd_, double kf_, FeedbackMode mode_) {
		this(kp_, ki_, kd_, kf_, -1, 1, 0, 0, mode_);
	}

	public PIDSettings(double kp_, double ki_, double kd_, double kf_) {
		this(kp_, ki_, kd_, kf_, -1, 1, 0, 0, FeedbackMode.LINEAR);
	}

	// TODO make this use rotation2d or length
	public PIDSettings(double kp_, double ki_, double kd_, double kf_, double minOutput_, double maxOutput_, double iZone_, double maxIAccum_, FeedbackMode mode_) {
		kp = kp_;
		ki = ki_;
		kd = kd_;
		kf = kf_;
		minOutput = minOutput_;
		maxOutput = maxOutput_;
		iZone = iZone_;
		maxIAccum = maxIAccum_;
		mode = mode_;
	}

	public FeedbackMode mode;
	public double kp, ki, kd, kf, minOutput, maxOutput, maxIAccum;
	public double iZone;
	public FeedForwardMode feedForwardMode;
	public FeedForwardBehavior feedForwardBehavior;

	public enum FeedbackMode {
		ANGULAR, LINEAR;
	}
}
