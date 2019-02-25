import static org.junit.Assert.assertEquals;

import com.github.kyled973.motionprofile.TriProfile;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.junit.jupiter.api.Test;

import frc.robot.lib.Logger;

public class TrapezoidalProfileTest {

	@Test
	public void testTrapProfile() {

		final int v_max = toTicks(1);
		final int accel = toTicks(10);
		final int dist = toTicks(0.5);
		final int start_vel = 0;
		final int end_vel = 0;

		TriProfile profile = new TriProfile(v_max, accel, dist, start_vel, end_vel);
	}

	public int toTicks(double rotations) {
		return (int) (rotations * 4096);
	}

}