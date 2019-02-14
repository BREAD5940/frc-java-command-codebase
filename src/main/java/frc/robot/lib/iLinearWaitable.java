package frc.robot.lib;

import org.ghrobotics.lib.mathematics.units.Length;

public interface iLinearWaitable extends iWaitable {

	abstract boolean withinTolerence();

	void setTolerence(Length tolerence);

}
