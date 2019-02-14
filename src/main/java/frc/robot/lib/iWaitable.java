package frc.robot.lib;

public interface iWaitable {

  abstract boolean withinTolerence();

  void setTolerence(Number tolerence);

}