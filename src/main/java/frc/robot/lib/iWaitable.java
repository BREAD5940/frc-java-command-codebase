package frc.robot.lib;

public interface iWaitable {

	abstract boolean withinTolerence();

	<T> void setTolerence(T tolerence);

}
