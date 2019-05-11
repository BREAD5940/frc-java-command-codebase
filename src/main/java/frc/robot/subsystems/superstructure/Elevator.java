package frc.robot.subsystems.superstructure;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.motors.ctre.FalconSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.SuperStructureConstants;
import frc.robot.lib.HalfBakedSubsystem;
import frc.robot.lib.Logger;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.InvertSettings;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;

/**
 * The elevator subsystem controls the elevator height
 * with talon hardware PID. Contains methods for converting
 * from encoder units to height, and vice versa too!
 *
 * @author Matthew Morley
 */
public class Elevator extends HalfBakedSubsystem {

	private DoubleSolenoid mSolenoid = Robot.getElevatorShifter();

	public DoubleSolenoid getSolenoid() {
		return Robot.getElevatorShifter();
	}

	public static enum EncoderMode {
		NONE, CTRE_MagEncoder_Relative;
	}

	public static enum ElevatorGear {
		LOW, HIGH;

		public static Value get(ElevatorGear state) {
			return (state == LOW) ? Value.kReverse : Value.kForward; // TODO check kforward state
		}
	}

	private ElevatorGear elevatorGear;
	private static final ElevatorGear kDefaultState = ElevatorGear.LOW; // default to nyooooommmmm mode

	public ElevatorGear getHatchMechState() {
		return (getSolenoid().get() == Value.kReverse) ? ElevatorGear.LOW : ElevatorGear.HIGH; // TODO check kforward state
	}

	public void setPistonState(ElevatorGear mReq) {
		getSolenoid().set(ElevatorGear.get(mReq));
	}

	// TODO check these quick maths, kTopOfInnerStage is used to switch gravity feedforward
	public static final Mass kCarriageMass = MassKt.getLb(/*30*/20).times(-1); // times negative one because CF springs
	public static final Mass kInnerStageMass = MassKt.getLb(6.5 + 30);

	public static final Length kTopOfInnerStage = LengthKt.getInch(22);

	public static final double KLowGearForcePerVolt = (512d / 12d /* newtons */) * 1.5;
	public static final double KHighGearForcePerVolt = (1500d / 12d /* newtons */ );

	public static final PIDSettings LOW_GEAR_PID = new PIDSettings(0.15, 0.0, 0, 0); // Low speed
	public static final PIDSettings HIGH_GEAR_PID = new PIDSettings(0.17, 0, 0, 0); // High speed
	private static final int kLowGearPIDSlot = 0; // low speed slot
	private static final int kHighGearPIDSlot = 1; // high gear slot
	public static final PIDSettings HIGH_GEAR_MOTION_MAGIC = new PIDSettings(0.45 * 1.2, 0, 0, 0.3, 5500, 10000); // High speed  // theoretical max is 8000 and 14000
	private static final int kHighGearMotionMagicPIDSlot = 3; // low speed slot

	protected Length m_heightTrim = LengthKt.getInch(0);

	public boolean elevatorZeroed = false;;

	public Length getHeightTrim() {
		return m_heightTrim;
	}

	public void setHeightTrim(Length new_) {
		this.m_heightTrim = new_;
	}

	public void jogHeightTrim(Length offset, boolean isUpwards) {
		var oldTrim = getHeightTrim();
		if (isUpwards) {
			setHeightTrim(oldTrim.plus(offset));
		} else {
			setHeightTrim(oldTrim.minus(offset));
		}
	}

	private FalconSRX<Length> mMaster;

	private FalconSRX<Length> mSlave1, mSlave2, mSlave3;

	private static ElevatorGear mCurrentGear;
	public static final ElevatorGear kDefaultGear = ElevatorGear.HIGH; // start in high speed

	public enum PositionMode {
		POSITION, MOTIONMAGIC;
	}

	NativeUnitLengthModel lengthModel = RobotConfig.elevator.elevatorModel;

	public Elevator(int masterPort, int slavePort1, int slavePort2, int slavePort3, EncoderMode mode, InvertSettings settings) {

		 super("Elevator");

		 var masterTalon = new TalonSRX(masterPort);

		if (mode == EncoderMode.CTRE_MagEncoder_Relative) {
			masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
			masterTalon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
			masterTalon.setSensorPhase(true);
		}

		// Quadrature Encoder of current
		// Talon
		masterTalon.configPeakOutputForward(+0.3, 0);
		masterTalon.configPeakOutputReverse(-0.3, 0);

		masterTalon.setSelectedSensorPosition(23000);

		masterTalon.setInverted(false);

		 mMaster = new FalconSRX<Length>(masterTalon, lengthModel);


		var slave1Talon = new TalonSRX(slavePort1);
		var slave12Talon = new TalonSRX(slavePort2);
		var slave1T3lon = new TalonSRX(slavePort3);

		slave1Talon.set(ControlMode.Follower, mMaster.getMotorController().getDeviceID());
		slave12Talon.set(ControlMode.Follower, mMaster.getMotorController().getDeviceID());
		slave1T3lon.set(ControlMode.Follower, mMaster.getMotorController().getDeviceID());

		slave1Talon.setInverted(InvertType.OpposeMaster);
		slave12Talon.setInverted(InvertType.FollowMaster);
		slave1T3lon.setInverted(InvertType.FollowMaster);

		mSlave1 = new FalconSRX<Length>(slave1Talon, lengthModel);
		mSlave2 = new FalconSRX<Length>(slave12Talon, lengthModel);
		mSlave3 = new FalconSRX<Length>(slave1T3lon, lengthModel);

		// mSlave2.setInverted(settings.slave2FollowerMode);
		// mSlave3.setInverted(settings.slave3FollowerMode);

		// mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		// mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		// setup PID gains
		setClosedLoopGains(kLowGearPIDSlot, LOW_GEAR_PID);
		setClosedLoopGains(kHighGearPIDSlot, HIGH_GEAR_PID);
		setMotionMagicGains();

		NativeUnit maxHeightRaw = lengthModel.toNativeUnitPosition(SuperStructureConstants.Elevator.top.times(0.95));
		getMaster().getMotorController().configForwardSoftLimitThreshold((int) maxHeightRaw.getValue(), 0);
		getMaster().getMotorController().configForwardSoftLimitEnable(true, 0);
		NativeUnit minHeightRaw = lengthModel.toNativeUnitPosition(SuperStructureConstants.Elevator.bottom);
		getMaster().getMotorController().configReverseSoftLimitThreshold(1, 0);
		getMaster().getMotorController().configReverseSoftLimitEnable(true, 0);

		mCurrentGear = kDefaultGear;
		setGear(kDefaultGear); // set shifter and closed loop slot
	}

	public void setPositionMode(PositionMode mode) {
		if (mode == PositionMode.MOTIONMAGIC) {
			configMotionMagicGains(HIGH_GEAR_MOTION_MAGIC);
		}
	}

	public void configMotionMagicGains(PIDSettings settings) {
		getMaster().getMotorController().configMotionCruiseVelocity((int) settings.motionMagicCruiseVelocity,0);
		getMaster().getMotorController().configMotionAcceleration((int) settings.motionMagicAccel, 0);
	}

	public FalconSRX<Length> getMaster() {
		return mMaster;
	}

	public List<FalconSRX<Length>> getAll() {
		return Arrays.asList(
				mMaster//, mSlave1, mSlave2, mSlave3);
		);
	}

	public NativeUnitLengthModel getModel() {
		return lengthModel;
	}

	public void setMotionMagicGains() {
		// Elevator elev = SuperStructure.getElevator();
		this.getMaster().getMotorController().configMotionAcceleration((int) (600 * 9 * 1.75), 0);
		this.getMaster().getMotorController().configMotionCruiseVelocity(4000, 0); // about 3500 theoretical max
		this.getMaster().getMotorController().configMotionSCurveStrength(0, 0);
		this.getMaster().getMotorController().config_kP(3, 0.45, 0);
		this.getMaster().getMotorController().config_kI(3, 0.0, 0);
		this.getMaster().getMotorController().config_kD(3, 4, 0);
		this.getMaster().getMotorController().config_kF(3, 0.24 * (500 / 400), 0);
		// this.getMaster().selectProfileSlot(3, 0);
		// this.getMaster().configClosedloopRamp(0.1);
	}

	public Length getHeight() {
		return LengthKt.getMeter(mMaster.getEncoder().getPosition());
	}

	public Velocity<Length> getVelocity() {
		return VelocityKt.getVelocity(LengthKt.getMeter(mMaster.getEncoder().getVelocity()));
	}

	public void setGear(ElevatorGear req) {
		this.mCurrentGear = req;
		this.setPistonState(req);
		if (req == ElevatorGear.LOW) {
			Robot.setElevatorShifter(true);
			getMaster().getMotorController().selectProfileSlot(kLowGearPIDSlot, 0);
			setMMGains(LOW_GEAR_PID);
		}
		if (req == ElevatorGear.HIGH) {
			Robot.setElevatorShifter(false);
			// getMaster().selectProfileSlot(kHighGearPIDSlot, 0);
			getMaster().getMotorController().selectProfileSlot(kHighGearMotionMagicPIDSlot, 0);
			setMMGains(HIGH_GEAR_MOTION_MAGIC);
		}
	}

	public double getFeetPerSecond() {
		return VelocityKt.getFeetPerSecond(getVelocity());
	}

	public double getFeet() {
		return getHeight().getFeet();
	}

	public Length getClosedLoopError() {
		if (getMaster().getMotorController().getControlMode() != ControlMode.PercentOutput) {
			return lengthModel.fromNativeUnitPosition(NativeUnitKt.getNativeUnits(mMaster.getMotorController().getClosedLoopError(0)));
		} else {
			return LengthKt.getFeet(0);
		}
	}

	public void zeroEncoder() {
		mMaster.getEncoder().resetPosition(0.0);
	}

	public void setNeutralMode(NeutralMode mode) {
		for (FalconSRX<Length> motor : getAll()) {
			motor.getMotorController().setNeutralMode(mode);
		}
	}

	public void stop() {
		getMaster().setNeutral();
	}

	public void setClosedLoopGains(int slot, double kp, double ki, double kd, double kf, double iZone, double maxIntegral, double minOut, double maxOut) {
		var motor = mMaster.getMotorController();

		motor.selectProfileSlot(slot, 0);
		motor.config_kP(slot, kp, 30);
		motor.config_kI(slot, ki, 30);
		motor.config_kD(slot, kd, 30);
		motor.config_kF(slot, kf, 30);
		motor.config_IntegralZone(slot, (int) Math.round(lengthModel.toNativeUnitPosition(LengthKt.getInch(iZone)).getValue()), 30);
		motor.configMaxIntegralAccumulator(slot, maxIntegral, 0);
		motor.configPeakOutputForward(maxOut, 0);
		motor.configPeakOutputReverse(minOut, 0);
	}

	public void setClosedLoopGains(int slot, PIDSettings config) {
		setClosedLoopGains(slot, config.kp, config.ki, config.kd, config.kf, config.iZone, config.maxIAccum, config.minOutput, config.maxOutput);
	}

	/**
	 * Set the talon as a target angle and feedforward throttle percent
	 */
	public void setPositionArbitraryFeedForward(Length setpoint, double feedForwardPercent) {
		setpoint = Util.limit(setpoint, SuperStructureConstants.Elevator.bottom, SuperStructureConstants.Elevator.top);

		getMaster().setUseMotionProfileForPosition(false);
		getMaster().setPosition(setpoint.getMeter(), feedForwardPercent);
	}

	/**
	 * Set the position of the elevator in motion magic mode mode
	 * @param setpoint how high to go
	 * @param feedForwardPercent how much throttle to add
	 */
	public void setMMArbitraryFeedForward(Length setpoint, double feedForwardPercent) {
		setpoint = Util.limit(setpoint, SuperStructureConstants.Elevator.bottom, SuperStructureConstants.Elevator.top);
		getMaster().getMotorController().selectProfileSlot(3, 0);
		// Logger.log("Elevator setpoint: " + setpoint.getInch() + " feedforward: " + feedForwardPercent + " current raw output: " + getMaster().getMotorOutputPercent());
//		getMaster().set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, feedForwardPercent);

		getMaster().setUseMotionProfileForPosition(true);
		getMaster().setPosition(setpoint.getMeter(), feedForwardPercent);
	}

	public void setMMGains(PIDSettings config) {
		Logger.log("Setting motion magic gains! Velocity: " + (int) config.motionMagicCruiseVelocity +
				" acceleration: " + (int) config.motionMagicAccel);
		getMaster().getMotorController().configMotionCruiseVelocity((int) config.motionMagicCruiseVelocity, 0);
		getMaster().getMotorController().configMotionAcceleration((int) config.motionMagicAccel, 0);
	}

	SuperStructureState requState = new SuperStructureState(new ElevatorState(LengthKt.getInch(27)));

	public void setPositionSetpoint(SuperStructureState requ_) {
		this.requState = requ_;
	}

	public void setPositionSetpoint(ElevatorState requ_) {
		setPositionSetpoint(new SuperStructureState(requ_));
	}

	@Override
	public void periodic() {
		var temp = requState;
		// temp.elevator.height = temp.elevator.height.plus(getHeightTrim()); // offset by trim
		var feedForwardVoltage = getVoltage(temp);

		// System.out.println("requ state: " + requState.getElevatorHeight().getInch() + " trim value: " + getHeightTrim().getInch() + " overall requ height: " + temp.getElevatorHeight().getInch());

		setMMArbitraryFeedForward(temp.getElevatorHeight(), feedForwardVoltage / 12);
	}

	/**
	 * Calculate the bexpected mass on the elevator given a state
	 * @param state current state, including game piece held
	 * @return mass accounting for game piece and inner stage
	 */
	public static double getVoltage(SuperStructureState state) {
		double voltage;
		if (state.getElevatorHeight().getInch() >= kTopOfInnerStage.getInch()) {
			voltage = 0.1;
		} else {
			voltage = -0.05;
		}

		return voltage * 12;

		// Mass total = MassKt.getLb(kCarriageMass.getLb());

		// if (state.getHeldPiece() == HeldPiece.HATCH) {
		// 	total = total.plus(SuperStructure.kHatchMass);
		// }
		// if (state.getHeldPiece() == HeldPiece.CARGO) {
		// 	total = total.plus(SuperStructure.kCargoMass);
		// }
		// if (state.getElevatorHeight().getInch() >= kTopOfInnerStage.getInch()) {
		// 	total = total.plus(kInnerStageMass);
		// }
		// double totalF = total.getKilogram() * 9.81 /* g */;
		// return (mCurrentGear == ElevatorGear.LOW) ? totalF / KLowGearForcePerVolt : totalF / KHighGearForcePerVolt;
	}

	public ElevatorState getCurrentState() {
		// System.out.println("height is " + getHeight().getInch());
		// Time time = TimeUnitsKt.getSecond(Timer.getFPGATimestamp());
		// Acceleration<Length> accel = AccelerationKt.getAcceleration(LengthKt.getMeter(
		// (getVelocity().getValue() - lastKnown.velocity.getValue()) / (time.getValue() - lastKnown.time.getValue())));
		return new ElevatorState(getHeight(), getVelocity());
	}

	 @Override
	 protected void initDefaultCommand() {

	 }

	@Override
	public void onDisable() {
		getMaster().setDutyCycle(0.0, 0.0);
		getMaster().setDutyCycle(0.0, 0.0);
		setHeightTrim(LengthKt.getInch(0));
	}
}