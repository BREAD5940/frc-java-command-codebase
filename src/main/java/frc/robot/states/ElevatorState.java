package frc.robot.states;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Time;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Timer;

public class ElevatorState {
	public Length height;
	public Time time;
	public Velocity<Length> velocity;
	public Acceleration<Length> acceleration;
	// public double feedForwardVoltage;
	// private Elevator elevator = SuperStructure.elevator;

	private Length kDefaultHeight = LengthKt.getFeet(0);
	private Velocity<Length> kDefaultVelocity = VelocityKt
			.getVelocity(LengthKt.getFeet(0));

	public ElevatorState(Length height_, Velocity<Length> velocity_,
			Acceleration<Length> accel_, Time time_) {
		height = height_;
		velocity = velocity_;
		acceleration = accel_;
		time = time_;
	}

	public ElevatorState(Length height_, Velocity<Length> velocity_) {
		height = height_;
		velocity = velocity_;
		acceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(0));
		time = TimeUnitsKt.getSecond(0);
	}

	public ElevatorState(Length height_, Velocity<Length> velocity_,
			Acceleration<Length> accel_) {
		height = height_;
		velocity = velocity_;
		acceleration = accel_;
		time = TimeUnitsKt.getMillisecond(System.currentTimeMillis());
	}

	public ElevatorState() {
		this(LengthKt.getFeet(0), VelocityKt.getVelocity(LengthKt.getFeet(0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(0)),
				TimeUnitsKt.getMillisecond(System.currentTimeMillis()));
	}

	public ElevatorState(boolean simulate) {
		this(LengthKt.getFeet(0), VelocityKt.getVelocity(LengthKt.getFeet(0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(0)),
				TimeUnitsKt.getMillisecond(System.currentTimeMillis()));
	}

	public ElevatorState(Length height) {
		this(height, VelocityKt.getVelocity(LengthKt.getFeet(0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(0)),
				TimeUnitsKt.getMillisecond(System.currentTimeMillis()));
	}

	public ElevatorState(Length height, boolean simulate) {
		this(height, VelocityKt.getVelocity(LengthKt.getFeet(0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(0)),
				TimeUnitsKt.getMillisecond(System.currentTimeMillis()));
	}

	public ElevatorState getNewState(ElevatorState lastState, Length height_,
			Velocity<Length> velocity_) {
		Acceleration<Length> accel = AccelerationKt
				.getAcceleration(LengthKt.getMeter((velocity_.getValue()
						- lastState.velocity.getValue())
						/ (Timer.getFPGATimestamp() - lastState.time.getValue())));
		return new ElevatorState(height_, velocity_, accel); // TODO fixme
	}

	public void setHeight(Length height_) {
		height = height_;
	}

	public Length getHeight() {
		return height;
	}

	public ElevatorState add(Length elevatorDelta) {
		return new ElevatorState(this.height.plus(elevatorDelta));
	}

	public ElevatorState getFrom(Length height) {
		return new ElevatorState(height);
	}

	public boolean isEqualTo(ElevatorState other) {
		return (this.acceleration.getValue() == other.acceleration.getValue()
				&& this.height.getFeet() == other.height.getFeet()
				&& this.velocity.getValue() == other.velocity.getValue());
	}

	public ElevatorState plus(Length offset) {
		return new ElevatorState(this.height.plus(offset));
	}

	@Override
	public String toString() {
		return "Height (inches): " + height.getInch();
	}

}
