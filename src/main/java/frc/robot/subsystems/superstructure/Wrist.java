package frc.robot.subsystems.superstructure;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Mass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.lib.PIDSettings;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.SuperStructureState;

/**
 * The Wrist contains some specific wrist safing stuff and extends
 * RotatingJoint. Also Overloads motino magic gains lol
 * 
 * @author Matthew Morley
 */
public class Wrist extends RotatingJoint {

	public Wrist(PIDSettings settings, int motorPort, FeedbackDevice sensor, double reduction, RoundRotation2d min, RoundRotation2d max, boolean invert, Length armLength, Mass mass) {
		super(settings, Arrays.asList(motorPort), sensor, reduction, min, max, invert, armLength, mass);
	}

	public RoundRotation2d getAbsoluteAngle() {
		var proximal = SuperStructure.getInstance().getElbow().getCurrentState().angle;
		var current = getCurrentState().angle;
		var corrected = current.plus(proximal.div(2));
		return corrected;
	}

	public RoundRotation2d getRelativeAngle() {
		var proximal = SuperStructure.getInstance().getElbow().getCurrentState().angle;
		var corrected = getAbsoluteAngle();
		var delta = corrected.minus(proximal);
		return delta;
	}

	public void requestAngle(RoundRotation2d reqAngle) {
		// System.out.println("requ angle: " + reqAngle.getDegree());
		// RoundRotation2d normed = reqAngle.minus(mCurrent.getElbowAngle());
		var normed = Util.limit(reqAngle, super.kMinAngle, super.kMaxAngle);
		super.getMaster().set(kDefaultControlMode, normed);
	}

	public void requestAngle(ControlMode mode, RoundRotation2d reqAngle, SuperStructureState mCurrent) {
		// System.out.println("requ angle: " + reqAngle.getDegree());

		// RoundRotation2d normed = reqAngle.minus(mCurrent.getElbowAngle());
		var normed = Util.limit(reqAngle, super.kMinAngle, super.kMaxAngle);
		super.getMaster().set(mode, normed);
	}

	public void requestAngle(RoundRotation2d reqAngle, double arbitraryFeedForward, SuperStructureState mCurrent) {
		// System.out.println("requ angle: " + reqAngle.getDegree());

		// RoundRotation2d normed = reqAngle.minus(mCurrent.getElbowAngle());
		var normed = Util.limit(reqAngle, super.kMinAngle, super.kMaxAngle);
		super.getMaster().set(kDefaultControlMode, normed, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
	}

	private PIDSettings kDefaultMotionMagicPidSettings = new PIDSettings(.1, 0, 0, 0.1, 1000, 1000);

	@Override
	public void setMotionMagicGains() {
		// Elevator elev = SuperStructure.getElevator();
		this.getMaster().configMotionAcceleration((int) (3500));
		this.getMaster().configMotionCruiseVelocity(2000); // about 3500 theoretical max
		this.getMaster().configMotionSCurveStrength(0);
		this.getMaster().config_kP(3, 3.5, 0);
		this.getMaster().config_kI(3, 0.001, 0);
		this.getMaster().config_kD(3, 0.0, 0);
		this.getMaster().config_kF(3, 0.4, 0);
		// this.getMaster().selectProfileSlot(3, 0);
		// this.getMaster().configClosedloopRamp(0.1);
	}

}
