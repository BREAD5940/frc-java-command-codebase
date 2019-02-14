package frc.robot.states;

import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

/**
 * right now this is basically just a pair of doubles, but maybe
 * in the future it could also have different constants about the 
 * superstructure based on the angles
 * 
 * this all assumes that '0' is straight forwards on both joints
 */
public class IntakeAngle {

	private RotatingArmState elbowAngle;
	private RotatingArmState wristAngle;
	// TODO maybe have sanity checking on the angles to make sure they're not out of bounds in the context of the intake?

	public IntakeAngle(RotatingArmState elbowAngle, RotatingArmState wristAngle) {
		this.wristAngle = wristAngle;
		this.elbowAngle = elbowAngle;
	}

	public double getMinHeight() {
		double min = 0; //TODO remove instan.

		if (elbowAngle.angle.getDegree() > 0) {
			min = 0;
		}
		return min;
	}

	public RotatingArmState getElbow() {
		return elbowAngle;
	}

	public RotatingArmState getWrist() {
		return wristAngle;
	}

	public boolean isEqualTo(IntakeAngle other) {
		return (this.elbowAngle.angle.getDegree() == other.elbowAngle.angle.getDegree()
				&& this.wristAngle.angle.getDegree() == other.wristAngle.angle.getDegree());
	}
}
