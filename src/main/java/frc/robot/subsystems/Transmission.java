package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.drivers.TalonSRXFactory;
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

		var mTalon = TalonSRXFactory.createDefaultTalon(masterPort);
		var sTalon = TalonSRXFactory.createPermanentSlaveTalon(slavePort, masterPort);

		this.side = side;

		if (mode == EncoderMode.CTRE_MagEncoder_Relative) {
			mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
			mTalon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 0);
		}
		// Quadrature Encoder of current
		// Talon
		mTalon.configPeakOutputForward(+1.0, 30);
		mTalon.configPeakOutputReverse(-1.0, 30);

		SmartDashboard.putBoolean("Is the " + side.toString() + " transmission inverted", isInverted);

		// if(isInverted == true) {
		// Logger.log("Making this inverted");
		mTalon.setInverted(isInverted);
		sTalon.setInverted(InvertType.FollowMaster);

		mTalon.configVoltageCompSaturation(12);
		mTalon.enableVoltageCompensation(true);

		sTalon.configVoltageCompSaturation(12);
		sTalon.enableVoltageCompensation(true);

		mMaster = new FalconSRX<Length>(mTalon, lengthModel);
		mSlave = new FalconSRX<Length>(sTalon, lengthModel);


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
		if (getMaster().getMotorController().getControlMode() != ControlMode.PercentOutput) {
			return lengthModel.fromNativeUnitPosition(NativeUnitKt.getNativeUnits(mMaster.getMotorController().getClosedLoopError(0)));
		} else {
			return LengthKt.getFeet(0);
		}
	}

	public void zeroEncoder() {
		mMaster.getEncoder().resetPosition(0);
	}

	public void setNeutralMode(NeutralMode mode) {
		for (FalconSRX<Length> motor : getAll()) {
			motor.getMotorController().setNeutralMode(mode);
		}
	}

	public void stop() {
		getMaster().setNeutral();
	}

	public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {

		var iMotorController = mMaster.getMotorController();

		iMotorController.config_kP(0, kp, 0);
		iMotorController.config_kI(0, ki, 0);
		iMotorController.config_kD(0, kd, 0);
		iMotorController.config_kF(0, kf, 0);
		iMotorController.config_IntegralZone(0, (int) Math.round(lengthModel.toNativeUnitPosition(LengthKt.getMeter(iZone)).getValue()), 30);
		iMotorController.configMaxIntegralAccumulator(0, maxIntegral, 0);
	}

}
