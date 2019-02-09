package frc.robot.lib;

public class ExecTimer {
	private long startTime;

	public ExecTimer() {
		startTime = System.currentTimeMillis();
	}

	public double time() {
		return (System.currentTimeMillis() - startTime) / 1000.0;
	}
}
