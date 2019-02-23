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
import frc.robot.SuperStructureConstants;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.lib.PIDSettings;
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

	public static final PIDSettings LOW_GEAR_PID = new PIDSettings(0.2 * 4, 0.1, 2, 0);
	public static final PIDSettings HIGH_GEAR_PID = new PIDSettings(0.05, 0, 0, 0);
	private static final int kLowGearPIDSlot = 0;
	private static final int kHighGearPIDSlot = 1;

	private FalconSRX<Length> mMaster;

	private FalconSRX<Length> mSlave1, mSlave2, mSlave3;

	private static ElevatorGear mCurrentGear;
	private static final ElevatorGear kDefaultGear = ElevatorGear.LOW;

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

		mMaster.setSelectedSensorPosition(0);

		mMaster.setInverted(settings.masterInverted);
		mSlave1.setInverted(settings.slave1FollowerMode);
		mSlave2.setInverted(settings.slave2FollowerMode);
		mSlave3.setInverted(settings.slave3FollowerMode);

		// mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, normalOpenOrClose);

		// setup PID gains
		setClosedLoopGains(kLowGearPIDSlot, LOW_GEAR_PID);
		setClosedLoopGains(kHighGearPIDSlot, HIGH_GEAR_PID);

		NativeUnit maxHeightRaw = lengthModel.toNativeUnitPosition(SuperStructureConstants.Elevator.top);
		getMaster().setSoftLimitForward(maxHeightRaw);
		getMaster().setSoftLimitForwardEnabled(true);
		NativeUnit minHeightRaw = lengthModel.toNativeUnitPosition(SuperStructureConstants.Elevator.bottom);
		getMaster().setSoftLimitReverse(minHeightRaw);
		getMaster().setSoftLimitReverseEnabled(true);


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

	public void setClosedLoopGains(int slot, double kp, double ki, double kd, double kf, double iZone, double maxIntegral, double minOut, double maxOut) {
		mMaster.selectProfileSlot(slot, 0);
		mMaster.config_kP(0, kp, 30);
		mMaster.config_kI(0, ki, 30);
		mMaster.config_kD(0, kd, 30);
		mMaster.config_kF(0, kf, 30);
		mMaster.config_IntegralZone(0, (int) Math.round(lengthModel.toNativeUnitPosition(LengthKt.getInch(iZone)).getValue()), 30);
		mMaster.configMaxIntegralAccumulator(0, maxIntegral, 0);
		mMaster.configPeakOutputForward(maxOut);
		mMaster.configPeakOutputReverse(minOut);
	}

	public void setClosedLoopGains(int slot, PIDSettings config) {
		setClosedLoopGains(slot, config.kp, config.ki, config.kd, config.kf, config.iZone, config.maxIAccum, config.minOutput, config.maxOutput);
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
			total = total.plus(SuperStructureConstants.kHatchMass);
		}
		if (state.getHeldPiece() == HeldPiece.CARGO) {
			total = total.plus(SuperStructureConstants.kCargoMass);
		}
		if (state.getElevatorHeight().getInch() >= kTopOfInnerStage.getInch()) {
			total = total.plus(kInnerStageMass);
		}
		double totalF = total.getKilogram() * 9.81 /* g */;
		return (mCurrentGear == ElevatorGear.LOW) ? totalF / KLowGearForcePerVolt : totalF / KHighGearForcePerVolt;
	}

	public ElevatorState getCurrentState() {
		// Time time = TimeUnitsKt.getSecond(Timer.getFPGATimestamp());
		// Acceleration<Length> accel = AccelerationKt.getAcceleration(LengthKt.getMeter(
		// (getVelocity().getValue() - lastKnown.velocity.getValue()) / (time.getValue() - lastKnown.time.getValue())));
		return new ElevatorState(getHeight(), getVelocity());
	}
}
