package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.AngularVelocity;
import frc.robot.lib.obj.HalfBakedRotatingSRX;
import frc.robot.lib.obj.RoundRotation2d;

public class RotatingJoint extends Subsystem {

	private ArrayList<HalfBakedRotatingSRX> motors = new ArrayList<HalfBakedRotatingSRX>();

	public Length kArmLength; // distance to COM of the arm

	public Mass kArmMass;

	public double kTorque = 0;

	public double kMotorResistance = 0.0896;

	private RotatingArmState mPeriodicIO = new RotatingArmState();

	public final RoundRotation2d kMinAngle, kMaxAngle;

	final ControlMode kDefaultControlMode = ControlMode.MotionMagic;

	// private PIDSettings pidSettings;

	private double mTicksPerRotation;

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
	public RotatingJoint(PIDSettings settings, int motorPort, FeedbackDevice sensor, double reduction, RoundRotation2d min, RoundRotation2d max, boolean invert, Length armLength, Mass mass) {
		this(settings, Arrays.asList(motorPort), sensor, reduction, min, max, invert, armLength, mass); //FIXME what should the default masterInvert ACTUALLY be?
	}

	/**
	 * Create an abstract rotating arm using PIDSettings, a List of ports and a FeedbackDevice.
	 * as of right now this only supports mag encoders. Remember to also have PID input and 
	 * output methods, as this is a PIDSubsystem
	 * @param PIDSettigns for the PIDSubsystem to use
	 * @param ports of talon CAN ports as a List
	 * @param sensor for the arm to use (ONLY MAG ENCODER TO USE)
	 * 
	 * @author Matthew Morley
	 */
	public RotatingJoint(PIDSettings settings, List<Integer> ports, FeedbackDevice sensor, double reduction, RoundRotation2d min, RoundRotation2d max, boolean masterInvert, Length armLength, Mass armMass) {    // super(name, settings.kp, settings.ki, settings.kd, settings.kf, 0.01f);

		kMinAngle = min;
		kMaxAngle = max;

		kArmLength = armLength;

		kArmMass = armMass;

		NativeUnit unitsPerRotation = NativeUnitKt.getSTU(0);

		// TODO add support for more sensors
		// if (sensor == FeedbackDevice.CTRE_MagEncoder_Relative) {
		// unitsPerRotation = NativeUnitKt.getSTU(4096).times(reduction);
		// }
		mTicksPerRotation = 4096 * reduction;

		// mRotationModel = new NativeUnitRotationModel(unitsPerRotation);

		// add all of our talons to the list
		for (Integer i : ports) {
			motors.add(new HalfBakedRotatingSRX(i.intValue(), mTicksPerRotation));
		}

		getMaster().setInverted(masterInvert);
		if (ports.size() > 1) {
			motors.get(1).set(ControlMode.Follower, ports.get(0));
			motors.get(1).setInverted(InvertType.OpposeMaster);
		}

		getMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		getMaster().configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 0);
		getMaster().setSensorPosition(RoundRotation2d.getDegree(0));
		setClosedLoopGains(0, settings);
		getMaster().configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		getMaster().configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		setMotionMagicGains();
	}

	public void setClosedLoopGains(int slot, double kp, double ki, double kd, double kf, double iZone, double maxIntegral, double minOut, double maxOut) {
		getMaster().selectProfileSlot(slot, 0);
		getMaster().config_kP(0, kp, 30);
		getMaster().config_kI(0, ki, 30);
		getMaster().config_kD(0, kd, 30);
		getMaster().config_kF(0, kf, 30);
		getMaster().config_IntegralZone(0, (int) Math.round(getMaster().getTicks(Rotation2dKt.getDegree(iZone))), 0);
		getMaster().configMaxIntegralAccumulator(0, maxIntegral, 0);
		getMaster().configPeakOutputForward(maxOut);
		getMaster().configPeakOutputReverse(minOut);
	}

	public void setGainMode(boolean isMotionMagic) {
		if (isMotionMagic)
			getMaster().selectProfileSlot(3, 0);
		if (!isMotionMagic)
			getMaster().selectProfileSlot(0, 0);
	}

	private PIDSettings kDefaultMotionMagicPidSettings = new PIDSettings(.1, 0, 0, 0.1, 1000, 1000);

	public void setMotionMagicGains() {
		// Elevator elev = SuperStructure.getElevator();
		this.getMaster().configMotionAcceleration((int) (4000));
		this.getMaster().configMotionCruiseVelocity(2000); // about 3500 theoretical max
		this.getMaster().configMotionSCurveStrength(0);
		this.getMaster().config_kP(3, 0.4, 0);
		this.getMaster().config_kI(3, 0.0, 0);
		this.getMaster().config_kD(3, 2.0, 0);
		this.getMaster().config_kF(3, 0.45, 0);
		this.getMaster().configClearPositionOnLimitF(true, 0);
		// this.getMaster().selectProfileSlot(3, 0);
		// this.getMaster().configClosedloopRamp(0.1);
	}

	public void setLimitSwitches() {
		var maxTicks = getMaster().getTicks(kMaxAngle);
		getMaster().configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		getMaster().configForwardSoftLimitThreshold(maxTicks);
		getMaster().configForwardSoftLimitEnable(true);
		getMaster().configClearPositionOnLimitF(false, 0);
	}

	public void setClosedLoopGains(int slot, PIDSettings config) {
		setClosedLoopGains(slot, config.kp, config.ki, config.kd, config.kf, config.iZone, config.maxIAccum, config.minOutput, config.maxOutput);
	}

	public void requestAngle(RoundRotation2d reqAngle) {
		reqAngle = Util.limit(reqAngle, kMinAngle, kMaxAngle);
		getMaster().set(kDefaultControlMode, reqAngle);
	}

	public void requestAngle(ControlMode mode, RoundRotation2d reqAngle) {
		reqAngle = Util.limit(reqAngle, kMinAngle, kMaxAngle);
		getMaster().set(mode, reqAngle);
	}

	/**
	 * Set the master talon to an anggle and arbitrary feed forward thorttle %
	 * @param reqAngle to to to
	 * @param feedForward extra throttle to apply
	 */
	public void requestAngleArbitraryFeedForward(RoundRotation2d reqAngle, double feedForward) {
		reqAngle = Util.limit(reqAngle, kMinAngle, kMaxAngle);
		getMaster().set(ControlMode.Position, reqAngle, DemandType.ArbitraryFeedForward, feedForward);
	}

	public void setSetpoint(double setpoint_) {}

	public void setSetpoint(Rotation2d _setpoint) {
		setSetpoint(Math.toDegrees(_setpoint.getValue()));
	}

	public RoundRotation2d getPosition() {
		return getMaster().getRotation2d();
	}

	/**
	 * Set the talon as a target angle and feedforward throttle percent
	 */
	public void setPositionArbitraryFeedForward(RoundRotation2d setpoint, double feedForwardPercent) {
		getMaster().set(ControlMode.Position, setpoint.getDegree(), DemandType.ArbitraryFeedForward, feedForwardPercent);
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
	public HalfBakedRotatingSRX getMaster() {
		return motors.get(0);
	}

	/**
	 * Return an ArrayList of all the falconSRXes
	 * @return motors... all of the motors
	 */
	public ArrayList<HalfBakedRotatingSRX> getAllMotors() {
		return motors;
	}

	/**
	 * Get the Rotation2d of the encoder of the master talon
	 * @return sensorPosition as a Rotation2d
	 */
	public RoundRotation2d getRotation() {
		return getMaster().getRotation2d();
	}

	public AngularVelocity getAngularVelocity() {
		return getMaster().getSensorVelocity();
	}

	/**
	 * Set the position of the sensor to the given Rotation2d pos_
	 * @param pos_ of the sensor as a Rotation2d
	 */
	public void setRotation(RoundRotation2d pos_) {
		getMaster().setSensorPosition(pos_);
	}

	public static class RotatingArmState {
		public RoundRotation2d angle;
		public AngularVelocity velocity;

		// public double feedForwardVoltage = 0;
		// public double pidOutput = 0;
		public RotatingArmState() {
			this(new RoundRotation2d(), new AngularVelocity());
		}

		public RotatingArmState(RoundRotation2d angle_) {
			this(angle_, new AngularVelocity());
		}

		public RotatingArmState(RoundRotation2d angle_, AngularVelocity velocity_) {
			this.angle = angle_;
			this.velocity = velocity_;
			// this.feedForwardVoltage = feedForwardVoltage;
		}

		public void setAngle(RoundRotation2d new__) {
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
		return getMaster().getRotation2d().getDegree();
	}

	@Override
	protected void initDefaultCommand() {}

	public boolean isWithinTolerance(RoundRotation2d tolerance, RoundRotation2d setpoint) {
		return Math.abs(getRotation().minus(setpoint).getDegree()) < tolerance.getDegree();
	}

}
