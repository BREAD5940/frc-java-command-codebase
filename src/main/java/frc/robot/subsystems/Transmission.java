package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.FalconFollowerSparkMax;
import frc.robot.lib.FalconSparkMax;
import frc.robot.lib.enums.TransmissionSide;

public class Transmission {
	public static enum EncoderMode {
		NONE, CTRE_MagEncoder_Relative;
	}

	// private FalconSRX<Length> mMaster;

	private FalconSparkMax mMaster;
	private FalconFollowerSparkMax mSlave;

	// private FalconSRX<Length> mSlave;

	private TransmissionSide side;
	private Encoder mEncoder;

	NativeUnitLengthModel lengthModel;// = driveTrain.LEFT_NATIVE_UNIT_LENGTH_MODEL;

	public Transmission(int masterPort, int slavePort, NativeUnitLengthModel lengthModel, Encoder encoder,
			TransmissionSide side, boolean isInverted) {

		// mMaster = new FalconSRX<Length>(masterPort, lengthModel,
		// TimeUnitsKt.getMillisecond(10));
		// mSlave = new FalconSRX<Length>(slavePort, lengthModel,
		// TimeUnitsKt.getMillisecond(10));
		mEncoder = encoder;
		this.lengthModel = lengthModel;
		mMaster = new FalconSparkMax(masterPort, MotorType.kBrushless, lengthModel);
		mSlave = new FalconFollowerSparkMax(slavePort, MotorType.kBrushless, mMaster, false);
		this.side = side;
		
		mMaster.enableVoltageCompensation(12);
		mSlave.enableVoltageCompensation(12);

		SmartDashboard.putBoolean("Is the " + side.toString() + " transmission inverted", isInverted);

		// if(isInverted == true) {
		// Logger.log("Making this inverted");
		mMaster.setInverted(isInverted);

		// }
	}

	public FalconSparkMax getMaster() {
		return mMaster;
	}

	public List<CANSparkMax> getAll() {
		return Arrays.asList(
				mMaster, mSlave);
	}


	public Length getDistance() {
		var rawDistance = NativeUnitKt.getNativeUnits(mEncoder.getDistance());
		var toReturn = lengthModel.fromNativeUnitPosition(rawDistance);
		return toReturn;
	}

	public Velocity<Length> getVelocity() {
		return mMaster.getVelocity();
	}

	public double getFeetPerSecond() {
		return VelocityKt.getFeetPerSecond(getVelocity());
	}

	public double getFeet() {
		return getDistance().getFeet();
	}

	public void setNeutralMode(IdleMode mode) {
		for (var motor : getAll()) {
			motor.setIdleMode(mode);
		}
	}

	public void stop() {
		getMaster().stopMotor();
	}

	public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
		mMaster.getController().setP(kp);
		mMaster.getController().setI(ki);
		mMaster.getController().setD(kd);
		// mMaster.getController().setIZone(IZone)
		// mMaster.config_kF(0, kf, 0);
		mMaster.getController().setIZone((int) Math.round(lengthModel.toNativeUnitPosition(LengthKt.getMeter(iZone)).getValue()), 30);
		// mMaster.configMaxIntegralAccumulator(0, maxIntegral, 0);
	}

	public void zeroEncoder() {
		mEncoder.reset();
		mMaster.getEncoder().setPosition(0);
	}

}
