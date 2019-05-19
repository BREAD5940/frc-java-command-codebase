import org.junit.jupiter.api.Test;

import frc.team254.Trajectory;
import frc.team254.TrajectoryGenerator;
import junit.framework.Assert;

public class TrapezoidalProfileTest {

	@Test
	public void testP2PTrapezoid() {
		test(0, 0, 120, TrajectoryGenerator.TrapezoidalStrategy);
	}

	// @Test
	// public void testP2PSCurve() {
	// 	test(0, 0, 100, TrajectoryGenerator.SCurvesStrategy);
	// }

	@Test
	public void AutoStrategyCurve() {
		test(0, 0, 100, TrajectoryGenerator.AutomaticStrategy);
	}

	/** 
	 * Test a trajectory with some default settings
	 * @param start_vel the velocity to start at 
	 * @param goal_vel the end velocity
	 * @param goal_distance how far to go
	 * @param strategy the strategy to follow
	 */
	static void test(double start_vel, double goal_vel, double goal_distance,
			TrajectoryGenerator.Strategy strategy) {
		TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();

		System.out.println(
				"=============================== Testing strategy " + strategy.toString()
						+ " ===============================");

		config.dt = .02;
		config.max_acc = 750.0;
		config.max_jerk = 2000.0;
		config.max_vel = 150.0;

		Trajectory traj = TrajectoryGenerator.generate(
				config,
				strategy,
				start_vel,
				0,
				goal_distance,
				goal_vel,
				0);

		// System.out.print(traj.toString());

		Trajectory.Segment last = traj.getSegment(traj.getNumSegments() - 1);
		Assert.assertFalse(Math.abs(last.pos - goal_distance) > 1.0);
		Assert.assertFalse(Math.abs(last.vel - goal_vel) > 1.0);
		Assert.assertFalse(Math.abs(last.heading) > 1.0);
	}

}
