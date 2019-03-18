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
import org.ghrobotics.lib.wrappers.FalconMotor;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConfig.driveTrain;
import frc.robot.lib.FalconFollowerSparkMax;
import frc.robot.lib.FalconPIDSparkMax;
import frc.robot.lib.FalconSparkMax;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.enums.TransmissionSide;

public class Transmission {
	public static enum EncoderMode {
		NONE, CTRE_MagEncoder_Relative;
	}

	// private FalconSRX<Length> mMaster;

	private FalconPIDSparkMax mMaster;
	private FalconFollowerSparkMax mSlave;

	// private FalconSRX<Length> mSlave;

	private TransmissionSide side;

	NativeUnitLengthModel lengthModel = driveTrain.LEFT_NATIVE_UNIT_LENGTH_MODEL;

	public Transmission(int masterPort, int slavePort, NativeUnitLengthModel lengthModel, Encoder encoder, TransmissionSide side, boolean isInverted) {

		// mMaster = new FalconSRX<Length>(masterPort, lengthModel, TimeUnitsKt.getMillisecond(10));
		// mSlave = new FalconSRX<Length>(slavePort, lengthModel, TimeUnitsKt.getMillisecond(10));

		mMaster = new FalconPIDSparkMax(masterPort, MotorType.kBrushless, lengthModel, encoder, new PIDSettings());
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

	public FalconPIDSparkMax getMaster() {
		return mMaster;
	}

	public List<CANSparkMax> getAll() {
		return Arrays.asList(
				mMaster, mSlave);
	}


	public Length getDistance() {
		return mMaster.getDistance();
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


	public void zeroEncoder() {
		
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
		mMaster.setKp(kp);
		mMaster.setKi(ki);
		mMaster.setKd(kd);
		// mMaster.config_kF(0, kf, 0);
		// mMaster.config_IntegralZone(0, (int) Math.round(lengthModel.toNativeUnitPosition(LengthKt.getMeter(iZone)).getValue()), 30);
		// mMaster.configMaxIntegralAccumulator(0, maxIntegral, 0);
	}

}
