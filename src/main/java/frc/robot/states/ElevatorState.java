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
  public double feedForwardVoltage;

  public ElevatorState(Length height_, Velocity<Length> velocity_, Acceleration<Length> accel_, double feedForwardVoltage_, Time time_) {
    height = height_;
    velocity = velocity_;
    acceleration = accel_;
    feedForwardVoltage = feedForwardVoltage_;
    time = time_;
  }

  public ElevatorState(Length height_, Velocity<Length> velocity_, Acceleration<Length> accel_, double feedForwardVoltage_) {
    height = height_;
    velocity = velocity_;
    acceleration = accel_;
    feedForwardVoltage = feedForwardVoltage_;
    time = TimeUnitsKt.getSecond(Timer.getFPGATimestamp());
  }

  public ElevatorState() {
    this(LengthKt.getFeet(0), VelocityKt.getVelocity(LengthKt.getFeet(0)), AccelerationKt.getAcceleration(LengthKt.getFeet(0)), 0f, 
            TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));
  }

  public ElevatorState getNewState(ElevatorState lastState, Length height_, Velocity<Length> velocity_,
          double feedForwardVoltage_) {
            Acceleration<Length> accel = AccelerationKt.getAcceleration( LengthKt.getMeter(( velocity_.getValue() - lastState.velocity.getValue()) / ( Timer.getFPGATimestamp() - lastState.time.getValue())));
            return new ElevatorState(height_, velocity_, accel, feedForwardVoltage_); // TODO fixme
  }
}