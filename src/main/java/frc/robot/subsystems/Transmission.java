package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConfig.driveTrain;
import frc.robot.lib.enums.TransmissionSide;
import org.ghrobotics.lib.motors.ctre.FalconSRX;

public class Transmission {
	public static enum EncoderMode {
		NONE, CTRE_MagEncoder_Relative;
	}

	private FalconSRX<Length> mMaster;

	private FalconSRX<Length> mSlave;

	private TransmissionSide side;

	NativeUnitLengthModel lengthModel = driveTrain.LEFT_NATIVE_UNIT_LENGTH_MODEL;

	public Transmission(int masterPort, int slavePort, EncoderMode mode, TransmissionSide side, boolean isInverted) {
		if (side == TransmissionSide.LEFT) {
			lengthModel = driveTrain.LEFT_NATIVE_UNIT_LENGTH_MODEL;
		} else {
			lengthModel = driveTrain.RIGHT_NATIVE_UNIT_LENGTH_MODEL;
		}

		mMaster = new FalconSRX<Length>(masterPort, lengthModel);
		mSlave = new FalconSRX<Length>(slavePort, lengthModel);

		this.side = side;

		if (mode == EncoderMode.CTRE_MagEncoder_Relative) {
			mMaster.getTalonSRX().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
			mMaster.getTalonSRX().configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 0);
		}
		mSlave.getTalonSRX().set(ControlMode.Follower, mMaster.getTalonSRX().getDeviceID());
		// Quadrature Encoder of current
		// Talon
		mMaster.getTalonSRX().configPeakOutputForward(+1.0, 30);
		mMaster.getTalonSRX().configPeakOutputReverse(-1.0, 30);

		SmartDashboard.putBoolean("Is the " + side.toString() + " transmission inverted", isInverted);

		// if(isInverted == true) {
		// Logger.log("Making this inverted");
		mMaster.getTalonSRX().setInverted(isInverted);
		mSlave.getTalonSRX().setInverted(InvertType.FollowMaster);

		mMaster.getTalonSRX().configVoltageCompSaturation(12);
		mMaster.getTalonSRX().enableVoltageCompensation(true);

		mSlave.getTalonSRX().configVoltageCompSaturation(12);
		mSlave.getTalonSRX().enableVoltageCompensation(true);
		// }
	}

	public FalconSRX<Length> getMaster() {
		return mMaster;
	}

	public List<FalconSRX<Length>> getAll() {
		return Arrays.asList(
				mMaster, mSlave);
	}

	public NativeUnitLengthModel getModel() {
		return lengthModel;
	}

	public Length getDistance() {
		return LengthKt.getMeter(mMaster.getEncoder().getPosition());
	}

	public Velocity<Length> getVelocity() {
		return VelocityKt.getVelocity(LengthKt.getMeter(mMaster.getEncoder().getVelocity()));
	}

	public double getFeetPerSecond() {
		return VelocityKt.getFeetPerSecond(getVelocity());
	}

	public double getFeet() {
		return getDistance().getFeet();
	}

	public Length getClosedLoopError() {
		if (getMaster().getTalonSRX().getControlMode() != ControlMode.PercentOutput) {
			return lengthModel.fromNativeUnitPosition(NativeUnitKt.getNativeUnits(mMaster.getTalonSRX().getClosedLoopError()));
		} else {
			return LengthKt.getFeet(0);
		}
	}

	public void zeroEncoder() {
		mMaster.getEncoder().resetPosition(0);
	}

	public void setNeutralMode(NeutralMode mode) {
		for (FalconSRX<Length> motor : getAll()) {
			motor.getTalonSRX().setNeutralMode(mode);
		}
	}

	public void stop() {
		getMaster().setNeutral();
	}

	public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
		mMaster.getTalonSRX().config_kP(0, kp, 0);
		mMaster.getTalonSRX().config_kI(0, ki, 0);
		mMaster.getTalonSRX().config_kD(0, kd, 0);
		mMaster.getTalonSRX().config_kF(0, kf, 0);
		mMaster.getTalonSRX().config_IntegralZone(0, (int) Math.round(lengthModel.toNativeUnitPosition(LengthKt.getMeter(iZone)).getValue()), 30);
		mMaster.getTalonSRX().configMaxIntegralAccumulator(0, maxIntegral, 0);
	}

}
