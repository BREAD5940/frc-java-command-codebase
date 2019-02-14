package frc.robot.subsystems.superstructure;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Time;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.lib.Logger;
import frc.robot.lib.PIDSettings;
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
public class Elevator /*extends Subsystem*/ {

	public static enum EncoderMode {
		NONE, CTRE_MagEncoder_Relative;
	}

	public static enum ElevatorGear {
		LOW, HIGH;
	}

	// TODO check these quick maths, kTopOfInnerStage is used to switch gravity feedforward
	public static final Mass kCarriageMass = MassKt.getLb(20);
	public static final Mass kInnerStageMass = MassKt.getLb(6.5);

	public static final Length kTopOfInnerStage = LengthKt.getInch(40);

	public static final double KLowGearForcePerVolt = 512d / 12d /* newtons */ ;
	public static final double KHighGearForcePerVolt = 1500d / 12d /* newtons */ ;

	public static final PIDSettings LOW_GEAR_PID = new PIDSettings(0.05, 0, 0, 0);
	public static final PIDSettings HIGH_GEAR_PID = new PIDSettings(0.05, 0, 0, 0);

	private FalconSRX<Length> mMaster;

	private FalconSRX<Length> mSlave1, mSlave2, mSlave3;

	private ElevatorGear mCurrentGear;
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
			mMaster.setSensorPhase(true);
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

		mCurrentGear = kDefaultGear;
		setGear(kDefaultGear);
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
		if (req == ElevatorGear.LOW) {
			Robot.setElevatorShifter(true);
			setClosedLoopGains(LOW_GEAR_PID);
		}
		if (req == ElevatorGear.HIGH) {
			Robot.setElevatorShifter(true);
			setClosedLoopGains(HIGH_GEAR_PID);
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
			return lengthModel.toModel(NativeUnitKt.getSTU(mMaster.getClosedLoopError()));
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

	public void setClosedLoopGains(double kp, double ki, double kd, double kf, Length iZone, double maxIntegral, double minOut, double maxOut) {
		mMaster.config_kP(0, kp, 30);
		mMaster.config_kI(0, ki, 30);
		mMaster.config_kD(0, kd, 30);
		mMaster.config_kF(0, kf, 30);
		mMaster.config_IntegralZone(0, (int) Math.round(lengthModel.fromModel(iZone).getValue()), 30);
		mMaster.configMaxIntegralAccumulator(0, maxIntegral, 0);
		mMaster.configPeakOutputForward(maxOut);
		mMaster.configPeakOutputReverse(minOut);
	}

	public void setClosedLoopGains(PIDSettings config) {
		setClosedLoopGains(config.kp, config.ki, config.kd, config.kf, config.iZone, config.maxIAccum, config.minOutput, config.maxOutput);
	}

	/**
	 * Set the talon as a target angle and feedforward throttle percent
	 */
	public void setPositionArbitraryFeedForward(Length setpoint, double feedForwardPercent) {
		getMaster().set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, feedForwardPercent);
	}

	/**
	 * Calculate the expected mass on the elevator given a state
	 * @param state current state, including game piece held
	 * @return mass accounting for game piece and inner stage
	 */
	public double getVoltage(SuperStructureState state) {
		Mass total = kCarriageMass;
		if (state.getHeldPiece() == HeldPiece.HATCH)
			total.plus(SuperStructure.kHatchMass);
		if (state.getHeldPiece() == HeldPiece.CARGO)
			total.plus(SuperStructure.kCargoMass);
		if (state.elevator.height.getValue() > kTopOfInnerStage.getValue())
			total.plus(kInnerStageMass);
		double totalF = total.getKilogram() * 9.81 /* g */;
		// ah shit we have force * force / volt, we want force * volt/force right? FIXME check my math with units
		return (mCurrentGear == ElevatorGear.LOW) ? KLowGearForcePerVolt / totalF : KHighGearForcePerVolt / totalF;
	}

	public ElevatorState getCurrentState(ElevatorState lastKnown) {
		Time time = TimeUnitsKt.getSecond(Timer.getFPGATimestamp());
		Acceleration<Length> accel = AccelerationKt.getAcceleration(LengthKt.getMeter(
				(getVelocity().getValue() - lastKnown.velocity.getValue()) / (time.getValue() - lastKnown.time.getValue())));
		return new ElevatorState(getHeight(), getVelocity(),
				accel, time);
	}
}
