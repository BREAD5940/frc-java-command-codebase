package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitRotationModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import frc.robot.lib.PIDSettings;

public class RotatingJoint /*extends Subsystem*/ {

	private ArrayList<FalconSRX<Rotation2d>> motors = new ArrayList<FalconSRX<Rotation2d>>();

	public Length kArmLength; // distance to COM of the arm

	public Mass kArmMass;

	public double kTorque = 0;

	public double kMotorResistance = 0.0896;

	private RotatingArmState mPeriodicIO = new RotatingArmState();

	// private PIDSettings pidSettings;

	private NativeUnitRotationModel mRotationModel;

	// public RotatingJoint(PIDSettings settings, int motorPort) {
	//   this(settings, motorPort, null, 0);
	// }

	/**
	 * Create an abstract rotating arm using PIDSettings, a List of ports and a FeedbackDevice.
	 * as of right now this only supports mag encoders. Remember to also have PID input and 
	 * output methods, as this is a PIDSubsystem
	 * @param PIDSettigns for the PIDSubsystem to use
	 * @param motorPort on the CAN Bus (for single talon arms)
	 * @param sensor for the arm to use (ONLY MAG ENCODER TO USE)
	 */
	public RotatingJoint(PIDSettings settings, int motorPort, FeedbackDevice sensor, boolean invert, Length armLength, Mass mass) {
		this(settings, Arrays.asList(motorPort), sensor, invert, armLength, mass); //FIXME what should the default masterInvert ACTUALLY be?
	}

	/**
	 * Create an abstract rotating arm using PIDSettings, a List of ports and a FeedbackDevice.
	 * as of right now this only supports mag encoders. Remember to also have PID input and 
	 * output methods, as this is a PIDSubsystem
	 * @param PIDSettigns for the PIDSubsystem to use
	 * @param ports of talon CAN ports as a List
	 * @param sensor for the arm to use (ONLY MAG ENCODER TO USE)
	 */
	public RotatingJoint(PIDSettings settings, List<Integer> ports, FeedbackDevice sensor, boolean masterInvert, Length armLength, Mass armMass) {    // super(name, settings.kp, settings.ki, settings.kd, settings.kf, 0.01f);

		kArmLength = armLength;

		kArmMass = armMass;

		NativeUnit unitsPerRotation = NativeUnitKt.getSTU(0);

		// TODO add support for more sensors
		// if (sensor == FeedbackDevice.CTRE_MagEncoder_Relative) {
		unitsPerRotation = NativeUnitKt.getSTU(4096);
		// }

		mRotationModel = new NativeUnitRotationModel(unitsPerRotation);

		// add all of our talons to the list
		for (Integer i : ports) {
			motors.add(new FalconSRX<Rotation2d>(i.intValue(), mRotationModel, TimeUnitsKt.getMillisecond(10)));
		}

		getMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		getMaster().configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 0);
		getMaster().setSensorPosition(Rotation2dKt.getDegree(0));
		setClosedLoopGains(0, settings);

	}


	public void setClosedLoopGains(int slot, double kp, double ki, double kd, double kf, double iZone, double maxIntegral, double minOut, double maxOut) {
		getMaster().selectProfileSlot(slot, 0);
		getMaster().config_kP(0, kp, 30);
		getMaster().config_kI(0, ki, 30);
		getMaster().config_kD(0, kd, 30);
		getMaster().config_kF(0, kf, 30);
		getMaster().config_IntegralZone(0, (int) Math.round(mRotationModel.fromModel(Rotation2dKt.getDegree(iZone)).getValue()), 0);
		getMaster().configMaxIntegralAccumulator(0, maxIntegral, 0);
		getMaster().configPeakOutputForward(maxOut);
		getMaster().configPeakOutputReverse(minOut);
	}

	public void setClosedLoopGains(int slot, PIDSettings config) {
		setClosedLoopGains(slot, config.kp, config.ki, config.kd, config.kf, config.iZone, config.maxIAccum, config.minOutput, config.maxOutput);
	}

	public void setSetpoint(double setpoint_) {}

	public void setSetpoint(Rotation2d _setpoint) {
		setSetpoint(Math.toDegrees(_setpoint.getValue()));
	}

	public Rotation2d getPosition() {
		return getMaster().getSensorPosition();
	}

	/**
	 * Set the talon as a target angle and feedforward throttle percent
	 */
	public void setPositionArbitraryFeedForward(Rotation2d setpoint, double feedForwardPercent) {
		getMaster().set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, feedForwardPercent);
	}

	/**
	 * Calculate the voltage based on a torque and velocity. Depreciated in favor of the 254 dcmotortransmission class
	 * @deprecated
	 * @param torque
	 * @param anglularVelocity
	 * @return
	 */
	@Deprecated
	public double calculateVoltage(double torque, Velocity<Rotation2d> anglularVelocity) {
		double tStatic = torque * kMotorResistance / kTorque;
		double tVelocity = kTorque * anglularVelocity.getValue(); // TODO make sure this is rad/s
		return tStatic + tVelocity;
	}

	/**
	 * Get the master talon of the rotating arm
	 */
	public FalconSRX<Rotation2d> getMaster() {
		return motors.get(0);
	}

	/**
	 * Return an ArrayList of all the falconSRXes
	 * @return motors... all of the motors
	 */
	public ArrayList<FalconSRX<Rotation2d>> getAllMotors() {
		return motors;
	}

	/**
	 * Get the Rotation2d of the encoder of the master talon
	 * @return sensorPosition as a Rotation2d
	 */
	public Rotation2d getRotation() {
		return getMaster().getSensorPosition();
	}

	public Velocity<Rotation2d> getAngularVelocity() {
		return getMaster().getSensorVelocity();
	}

	/**
	 * Set the position of the sensor to the given Rotation2d pos_
	 * @param pos_ of the sensor as a Rotation2d
	 */
	public void setRotation(Rotation2d pos_) {
		getMaster().setSensorPosition(pos_);
	}

	public static class RotatingArmState {
		public Rotation2d angle;
		public Velocity<Rotation2d> velocity;

		// public double feedForwardVoltage = 0;
		// public double pidOutput = 0;
		public RotatingArmState() {
			this(Rotation2dKt.getDegree(0), VelocityKt.getVelocity(Rotation2dKt.getDegree(0)));
		}

		public RotatingArmState(Rotation2d angle_) {
			this(angle_, VelocityKt.getVelocity(Rotation2dKt.getDegree(0)));
		}

		public RotatingArmState(Rotation2d angle_, Velocity<Rotation2d> velocity_) {
			this.angle = angle_;
			this.velocity = velocity_;
			// this.feedForwardVoltage = feedForwardVoltage;
		}

		public void setAngle(Rotation2d new__) {
			this.angle = new__;
		}

		@Override
		public String toString() {
			return angle.getDegree() + "";
			// return "hellothere";
		}
	}

	public RotatingArmState getCurrentState() {
		return new RotatingArmState(getRotation(), getAngularVelocity());
	}

	public double getDegrees() {
		return Math.toDegrees(getMaster().getSensorPosition().getValue());
	}
}
