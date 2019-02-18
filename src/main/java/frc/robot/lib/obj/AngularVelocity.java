package frc.robot.lib.obj;

import org.ghrobotics.lib.mathematics.units.Time;

public class AngularVelocity {
  
  RoundRotation2d rotPerSec;

  public AngularVelocity(RoundRotation2d rotationPerSecond){
    this.rotPerSec = rotationPerSecond;
  }

  public AngularVelocity() {
    this(new RoundRotation2d());
  }

  public AngularVelocity(RoundRotation2d rot, Time t){
    this(rot.times(t.getSecond()));
  }
}