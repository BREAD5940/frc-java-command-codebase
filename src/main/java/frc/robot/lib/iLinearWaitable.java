package frc.robot.lib;

import org.ghrobotics.lib.mathematics.units.Length;

import frc.robot.states.ElevatorState;

public interface iLinearWaitable {

	abstract boolean withinTolerence(ElevatorState current_, ElevatorState target_);

	abstract void setTolerence(Length tolerence);

}
