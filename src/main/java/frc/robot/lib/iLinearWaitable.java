package frc.robot.lib;

import org.ghrobotics.lib.mathematics.units.Length;

public interface iLinearWaitable {

	abstract boolean withinTolerence();

	abstract void setTolerence(Length tolerence);

}
