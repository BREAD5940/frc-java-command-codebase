package frc.robot.lib;

import org.ghrobotics.lib.mathematics.units.Rotation2d;

public interface iRotatingWaitable extends iWaitable {

	abstract boolean withinTolerence();

	void setTolerence(Rotation2d tolerence);

}
