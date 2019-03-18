package frc.robot.lib;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.FalconMotor;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;

public abstract class FalconPIDSparkMax extends CANSparkMax implements FalconMotor<Length> {

	private Encoder encoder;
	private PIDController controller;
	private double arbitraryFeedForwardValue = 0;
	// private int ticksPerRev;
	// private Length diameter;
	private NativeUnitLengthModel model;

	public FalconPIDSparkMax(int port, MotorType type, NativeUnitLengthModel model, Encoder encoder, PIDSettings settings) {
		super(port, type);
		this.encoder = encoder;
		encoder.setPIDSourceType(PIDSourceType.kRate);
		controller = new PIDController(settings.kp, settings.ki, settings.kd, encoder, this, 0.01);

		this.model = model;

		setNeutral();
		setSmartCurrentLimit(FalconSparkMax.kDefaultStallCurrent, FalconSparkMax.kFreeCurrent);
	}

	public void setKp(double kp) {
		controller.setP(kp);
	}

	public void setKi(double ki) {
		controller.setP(ki);
	}

	public void setKd(double kd) {
		controller.setP(kd);
	}

	public PIDController getController() {
		return controller;
	}

	public synchronized void setVelocity(Velocity<Length> speed) {
		setVelocityAndArbitraryFeedForward(speed, 0);
	}

	public synchronized void setNeutral() {
		super.set(0);
	}

	public synchronized void setVelocityAndArbitraryFeedForward(Velocity<Length> speed, double arbitraryFeedForward) {

		if (!controller.isEnabled())
			controller.enable();

		var rawVel = model.toNativeUnitVelocity(speed);

		controller.setSetpoint(rawVel.getValue());

		Logger.log("controller err is " + controller.getError());

		Logger.log("controller is enabled? " + controller.isEnabled());

		arbitraryFeedForwardValue = arbitraryFeedForward;
	}

	public synchronized void setPercentOutputArbFF(double percent, double arbFF) {
		this.arbitraryFeedForwardValue = arbFF;
		if (controller.isEnabled())
			controller.disable();
		set(percent + arbFF);
	}

	public synchronized void setPercentOutput(double demand) {
		setPercentOutputArbFF(demand, 0);
	}

	public Length getDistance() {
		var rawDistance = encoder.getDistance();
		var distance = model.fromNativeUnitPosition(NativeUnitKt.getNativeUnits(rawDistance));
		return distance;
	}

	@Override
	public Velocity<Length> getVelocity() {
		double ticks = encoder.getRate();
		var velocity = model.fromNativeUnitVelocity(VelocityKt.getVelocity(NativeUnitKt.getNativeUnits(ticks)));
		return velocity;
	}

	public double getRawPos() {
		return encoder.pidGet();
	}

	@Override
	public synchronized void pidWrite(double output) {
		// System.out.println("setting pwm controller to " + (output + arbitraryFeedForwardValue));
		set(output + arbitraryFeedForwardValue);
	}

	public ErrorCode configOpenloopRamp(double d) {
		return null;
		//FIXME this is fine
	}

}
