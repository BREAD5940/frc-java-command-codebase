package frc.robot.lib;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VoltKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.FalconMotor;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

public class FalconSparkMax extends CANSparkMax implements FalconMotor<Length> {

	// NativeUnitLengthModel model;
	CANEncoder neoEncoder;
	Length wheelDiameter;
	double reduction;
	CANPIDController controller;
	int m_currentPIDSlot = 0;
	NativeUnitLengthModel mModel;

	public static final int kDefaultStallCurrent = 30;
	public static final int kFreeCurrent = 60;

	/**
	 * Make a CAN Spark Max which can utilize the built in PID controller. z
	 * @param port
	 * @param type the type of motor
	 * @param model the native unit length model assuming RPM and revolutions
	 */
	public FalconSparkMax(int port, MotorType type, NativeUnitLengthModel model) {
		super(port, type);
		neoEncoder = new CANEncoder(this);
		controller = super.getPIDController();
		// this.reduction = reduction;
		this.mModel = model;

		setSmartCurrentLimit(kDefaultStallCurrent, kFreeCurrent);

		getEncoder().setVelocityConversionFactor(60 * 4096);
		getEncoder().setPositionConversionFactor(4096);
	}

	public CANPIDController getController() {
		return controller;
	}

	public void setClosedLoopGains(int slot, PIDSettings settings) {
		controller.setP(settings.kp, slot);
		controller.setI(settings.ki, slot);
		controller.setD(settings.kd, slot);
		controller.setFF(settings.kf, slot);
		m_currentPIDSlot = slot;
	}

	public void setNeutral() {
		super.disable();
	}

	@Override
	public double getPercentOutput() {
		return super.getAppliedOutput();
	}

	@Override
	public Velocity<Length> getVelocity() {
		// var rawRPM = neoEncoder.getPosition() / reduction;
		// var sufaceSpeed = wheelDiameter.times(Math.PI).times(rawRPM);
		// var toReturn = VelocityKt.getVelocity(sufaceSpeed);
		// return toReturn;
		return mModel.fromNativeUnitVelocity(VelocityKt.getVelocity(NativeUnitKt.getNativeUnits(neoEncoder.getVelocity())));
	}

	public Length getDistance() {
		var rawRevs = neoEncoder.getPosition();
		var distance = mModel.fromNativeUnitPosition(NativeUnitKt.getNativeUnits(rawRevs));
		return distance;
	}

	@Override
	public Volt getVoltageOutput() {
		var toReturn = super.getAppliedOutput() / super.getBusVoltage();
		return VoltKt.getVolt(toReturn);
	}

	@Override
	public void setPercentOutput(double arg0) {
		super.set(arg0);
		
	}

	@Override

	public void setVelocity(Velocity<Length> arg0) {
		var ticksPerSec = mModel.toNativeUnitVelocity(arg0.getValue());
		System.out.println("Setting Spark max PID Controller to " +ticksPerSec + " ticks per sec!");
		controller.setReference(ticksPerSec, ControlType.kVelocity);
	}

	@Override
	public void setVelocityAndArbitraryFeedForward(Velocity<Length> arg0, double arg1) {
		double setpoint = mModel.toNativeUnitVelocity(arg0.getValue());
		controller.setReference(setpoint, ControlType.kVelocity, m_currentPIDSlot, arg1);
	}

}
