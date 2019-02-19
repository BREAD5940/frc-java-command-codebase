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
 * Add your docs here.
 */
public class Wrist extends RotatingJoint {

	public Wrist(PIDSettings settings, int motorPort, FeedbackDevice sensor, double reduction, RoundRotation2d min, RoundRotation2d max, boolean invert, Length armLength, Mass mass) {
		super(settings, Arrays.asList(motorPort), sensor, reduction, min, max, invert, armLength, mass);
	}

	public void requestAngle(RoundRotation2d reqAngle, SuperStructureState mCurrent) {
		RoundRotation2d normed = reqAngle.minus(mCurrent.getElbowAngle());
		normed = Util.limit(reqAngle, super.kMinAngle, super.kMaxAngle);
		super.getMaster().set(ControlMode.Position, normed);
	}

	public void requestAngle(RoundRotation2d reqAngle, double arbitraryFeedForward, SuperStructureState mCurrent) {
		RoundRotation2d normed = reqAngle.minus(mCurrent.getElbowAngle());
		normed = Util.limit(reqAngle, super.kMinAngle, super.kMaxAngle);
		super.getMaster().set(ControlMode.Position, normed, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
	}

}
