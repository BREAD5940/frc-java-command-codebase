package frc.robot.subsystems.superstructure;

import java.util.Arrays;
import java.util.List;

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
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.InvertSettings;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;

/**
 * The elevator subsystem controls the elevator height
 * with talon hardware PID. Contains methods for converting
 * from encoder units to height, and vice versa too!
 * 
 * @author Matthew Morley
 */
public class Elevator /*extends Subsystem*/ {

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
	public static final Mass kCarriageMass = MassKt.getLb(30); // TODO add in the intake and stuff
	public static final Mass kInnerStageMass = MassKt.getLb(6.5);

	public static final Length kTopOfInnerStage = LengthKt.getInch(40);

	public static final double KLowGearForcePerVolt = (512d / 12d /* newtons */) * 1.5;
	public static final double KHighGearForcePerVolt = (1500d / 12d /* newtons */ );

	private static final int mHighGearUnitsPer100ms = 8192 / 10;
	private static final int mHighGearKf = (int) 1023 / mHighGearUnitsPer100ms;
	public static final PIDSettings LOW_GEAR_PID = new PIDSettings(0.15, 0, 0, 0); // high torque low speed
	public static final PIDSettings HIGH_GEAR_PID = new PIDSettings(0.15, 0, 0, mHighGearKf, 8192 /* raw units per 100ms */, 8192 / 10 /* raw units per 100ms */); // low torque high speed
	private static final int kLowGearPIDSlot = 0;
	private static final int kHighGearPIDSlot = 1;

	private FalconSRX<Length> mMaster;

	private FalconSRX<Length> mSlave1, mSlave2, mSlave3;

	private ElevatorGear mCurrentGear;
	private static final ElevatorGear kDefaultGear = ElevatorGear.HIGH;

	NativeUnitLengthModel lengthModel = RobotConfig.elevator.elevatorModel;

	public Elevator(int masterPort, int slavePort1, int slavePort2, int slavePort3, EncoderMode mode, InvertSettings settings) {

		mMaster = new FalconSRX<Length>(masterPort, lengthModel, TimeUnitsKt.getMillisecond(10));
		mSlave1 = new FalconSRX<Length>(slavePort1, lengthModel, TimeUnitsKt.getMillisecond(10));
		mSlave2 = new FalconSRX<Length>(slavePort2, lengthModel, TimeUnitsKt.getMillisecond(10));
		mSlave3 = new FalconSRX<Length>(slavePort3, lengthModel, TimeUnitsKt.getMillisecond(10));

		if (mode == EncoderMode.CTRE_MagEncoder_Relative) {
			mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
			mMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
			mMaster.setSensorPhase(false);
		}

		mSlave1.set(ControlMode.Follower, mMaster.getDeviceID());
		mSlave2.set(ControlMode.Follower, mMaster.getDeviceID());
		mSlave3.set(ControlMode.Follower, mMaster.getDeviceID());

		// Quadrature Encoder of current
		// Talon
		mMaster.configPeakOutputForward(+1.0, 0);
		mMaster.configPeakOutputReverse(-1.0, 0);

		zeroEncoder();

		mMaster.setInverted(settings.masterInverted);
		mSlave1.setInverted(settings.slave1FollowerMode);
		mSlave2.setInverted(settings.slave2FollowerMode);
		mSlave3.setInverted(settings.slave3FollowerMode);

		// mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, normalOpenOrClose);

		// setup PID gains
		setClosedLoopGains(kLowGearPIDSlot, LOW_GEAR_PID);
		setClosedLoopGains(kHighGearPIDSlot, HIGH_GEAR_PID);

		// NativeUnit maxHeightRaw = lengthModel.toNativeUnitPosition(SuperstructurePlanner.top);
		// getMaster().setSoftLimitForward(maxHeightRaw);
		// getMaster().setSoftLimitForwardEnabled(true);
		// NativeUnit minHeightRaw = lengthModel.toNativeUnitPosition(SuperstructurePlanner.bottom.minus(LengthKt.getInch(0.5)));
		// getMaster().setSoftLimitReverse(minHeightRaw);
		// getMaster().setSoftLimitReverseEnabled(false);

		mCurrentGear = kDefaultGear;
		setGear(kDefaultGear); // set shifter and closed loop slot
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

	public Length getHeight() {
		return mMaster.getSensorPosition();
	}

	public Velocity<Length> getVelocity() {
		return mMaster.getSensorVelocity();
	}

	public void setGear(ElevatorGear req) {
		this.mCurrentGear = req;
		this.setPistonState(req);
		if (req == ElevatorGear.LOW) {
			Robot.setElevatorShifter(true);
			getMaster().selectProfileSlot(kLowGearPIDSlot, 0);
		}
		if (req == ElevatorGear.HIGH) {
			Robot.setElevatorShifter(false);
			getMaster().selectProfileSlot(kHighGearPIDSlot, 0);
		}
	}

	public double getFeetPerSecond() {
		return VelocityKt.getFeetPerSecond(getVelocity());
	}

	public double getFeet() {
		return getHeight().getFeet();
	}

	public Length getClosedLoopError() {
		if (getMaster().getControlMode() != ControlMode.PercentOutput) {
			return lengthModel.fromNativeUnitPosition(NativeUnitKt.getNativeUnits(mMaster.getClosedLoopError()));
		} else {
			return LengthKt.getFeet(0);
		}
	}

	public boolean isWithinTolerence(SuperStructureState mCurrent) {
		return isWithinTolerence(mCurrent.getElevator());
	}

	public boolean isWithinTolerence(ElevatorState mCurrent) {
		return isWithinTolerence(mCurrent.height);
	}

	public boolean isWithinTolerence(Length mCurrent) {
		return (Math.abs(getClosedLoopError().getInch()) < RobotConfig.elevator.elevatorTolerences.position_tolerence.getInch());
	}

	public void requestClosedLoop(ControlMode mode, Length req_) {
		requestClosedLoop(mode, req_, DemandType.Neutral, 0);
	}

	public void requestClosedLoop(ControlMode mode, Length req_, DemandType type, double arg2) {
		req_ = Util.limit(req_, RobotConfig.elevator.elevator_minimum_height, RobotConfig.elevator.elevator_maximum_height);
		System.out.printf("requesting a move to %s in control mode %s with a demand of %s and value %s", req_.getInch(), mode.name(), arg2, type.name());
		getMaster().set(mode, req_, type, arg2);
	}

	public void zeroEncoder() {
		mMaster.setSensorPosition(LengthKt.getMeter(0));
	}

	public void setNeutralMode(NeutralMode mode) {
		for (FalconSRX<Length> motor : getAll()) {
			motor.setNeutralMode(mode);
		}
	}

	public void stop() {
		getMaster().set(ControlMode.PercentOutput, 0);
	}

	public void setClosedLoopGains(int slot, double kp, double ki, double kd, double kf, double mmVel, double mmAccel, double iZone, double maxIntegral, double minOut, double maxOut) {
		mMaster.selectProfileSlot(slot, 0);
		mMaster.config_kP(slot, kp, 0);
		mMaster.config_kI(slot, ki, 0);
		mMaster.config_kD(slot, kd, 0);
		mMaster.config_kF(slot, kf, 0);
		mMaster.configMotionCruiseVelocity((int) mmVel);
		mMaster.configMotionAcceleration((int) mmAccel);
		mMaster.configMotionSCurveStrength(0);
		mMaster.config_IntegralZone(slot, (int) Math.round(lengthModel.toNativeUnitPosition(LengthKt.getInch(iZone)).getValue()), 0);
		mMaster.configMaxIntegralAccumulator(slot, maxIntegral, 0);
		mMaster.configPeakOutputForward(maxOut);
		mMaster.configPeakOutputReverse(minOut);
	}

	public void setClosedLoopGains(int slot, PIDSettings config) {
		setClosedLoopGains(slot, config.kp, config.ki, config.kd, config.kf, config.motionMagicCruiseVelocity, config.motionMagicAccel, config.iZone, config.maxIAccum, config.minOutput, config.maxOutput);
	}

	/**
	 * Set the talon as a target angle and feedforward throttle percent
	 */
	public void setPositionArbitraryFeedForward(Length setpoint, double feedForwardPercent) {
		getMaster().set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, feedForwardPercent);
	}

	/**
	 * Calculate the bexpected mass on the elevator given a state
	 * @param state current state, including game piece held
	 * @return mass accounting for game piece and inner stage
	 */
	public static double getVoltage(SuperStructureState state) {
		Mass total = MassKt.getLb(kCarriageMass.getLb());

		if (state.getHeldPiece() == HeldPiece.HATCH) {
			total = total.plus(SuperStructure.kHatchMass);
		}
		if (state.getHeldPiece() == HeldPiece.CARGO) {
			total = total.plus(SuperStructure.kCargoMass);
		}
		if (state.getElevatorHeight().getInch() >= kTopOfInnerStage.getInch()) {
			total = total.plus(kInnerStageMass);
		}
		double totalF = total.getKilogram() * 9.81 /* g */;
		return (SuperStructure.getElevator().mCurrentGear == ElevatorGear.LOW) ? totalF / KLowGearForcePerVolt : totalF / KHighGearForcePerVolt;
	}

	public ElevatorState getCurrentState() {
		// Time time = TimeUnitsKt.getSecond(Timer.getFPGATimestamp());
		// Acceleration<Length> accel = AccelerationKt.getAcceleration(LengthKt.getMeter(
		// (getVelocity().getValue() - lastKnown.velocity.getValue()) / (time.getValue() - lastKnown.time.getValue())));
		return new ElevatorState(getHeight(), getVelocity());
	}

	// @Log.Graph(name = "Elevator Height", visibleTime = 15)
	public double getHeightInches() {
		return getHeight().getInch();
	}

	// @Log.Graph(name = "Elevator Velocity", visibleTime = 15)
	public double getVelocityInSec() {
		return VelocityKt.getInchesPerSecond(getVelocity());
	}

	/**
	 * @return the mSolenoid
	 */
	public DoubleSolenoid getmSolenoid() {
		return mSolenoid;
	}

	/**
	 * @param mSolenoid the mSolenoid to set
	 */
	public void setmSolenoid(DoubleSolenoid mSolenoid) {
		this.mSolenoid = mSolenoid;
	}

	/**
	 * @return the elevatorGear
	 */
	public ElevatorGear getElevatorGear() {
		return elevatorGear;
	}

	/**
	 * @param elevatorGear the elevatorGear to set
	 */
	public void setElevatorGear(ElevatorGear elevatorGear) {
		this.elevatorGear = elevatorGear;
	}

	/**
	 * @return the mMaster
	 */
	public FalconSRX<Length> getmMaster() {
		return mMaster;
	}

	/**
	 * @param mMaster the mMaster to set
	 */
	public void setmMaster(FalconSRX<Length> mMaster) {
		this.mMaster = mMaster;
	}

	/**
	 * @return the mSlave1
	 */
	public FalconSRX<Length> getmSlave1() {
		return mSlave1;
	}

	/**
	 * @param mSlave1 the mSlave1 to set
	 */
	public void setmSlave1(FalconSRX<Length> mSlave1) {
		this.mSlave1 = mSlave1;
	}

	/**
	 * @return the mSlave2
	 */
	public FalconSRX<Length> getmSlave2() {
		return mSlave2;
	}

	/**
	 * @param mSlave2 the mSlave2 to set
	 */
	public void setmSlave2(FalconSRX<Length> mSlave2) {
		this.mSlave2 = mSlave2;
	}

	/**
	 * @return the mSlave3
	 */
	public FalconSRX<Length> getmSlave3() {
		return mSlave3;
	}

	/**
	 * @param mSlave3 the mSlave3 to set
	 */
	public void setmSlave3(FalconSRX<Length> mSlave3) {
		this.mSlave3 = mSlave3;
	}

	/**
	 * @return the mCurrentGear
	 */
	public ElevatorGear getmCurrentGear() {
		return mCurrentGear;
	}

	/**
	 * @param mCurrentGear the mCurrentGear to set
	 */
	public void setmCurrentGear(ElevatorGear mCurrentGear) {
		this.mCurrentGear = mCurrentGear;
	}

	/**
	 * @return the lengthModel
	 */
	public NativeUnitLengthModel getLengthModel() {
		return lengthModel;
	}

	/**
	 * @param lengthModel the lengthModel to set
	 */
	public void setLengthModel(NativeUnitLengthModel lengthModel) {
		this.lengthModel = lengthModel;
	}

}
