package frc.robot.lib;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;

public interface iLinearWaitable extends iWaitable {

  abstract boolean withinTolerence();

  void setTolerence(Translation2d tolerence);

}