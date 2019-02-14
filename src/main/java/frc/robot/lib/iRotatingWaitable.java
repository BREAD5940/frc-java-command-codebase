package frc.robot.lib;

import org.ghrobotics.lib.mathematics.units.Rotation2d;

public interface iRotatingWaitable {

	abstract boolean withinTolerence();

	void setTolerence(Rotation2d tolerence);

}
