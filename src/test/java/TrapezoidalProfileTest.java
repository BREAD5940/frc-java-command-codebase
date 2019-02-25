import org.junit.jupiter.api.Test;

import frc.kyled973.motionprofile.Profile;
import frc.kyled973.motionprofile.TrapProfile;

public class TrapezoidalProfileTest {

	@Test
	public void testTrapProfile() {

		final int v_max = toTicks(1);
		final int accel = toTicks(10);
		final int dist = toTicks(0.5);
		final int start_vel = 0;
		final int end_vel = 0;

		final Profile profile = Profile.getVelProfile(v_max, accel, dist, start_vel, end_vel);



	}

	public int toTicks(double rotations) {
		return (int) (rotations * 4096);
	}

}