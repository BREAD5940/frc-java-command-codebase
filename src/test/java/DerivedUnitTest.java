import static org.junit.Assert.assertEquals;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.junit.jupiter.api.Test;

import frc.robot.lib.Logger;
import frc.robot.lib.motion.Util;

public class DerivedUnitTest {

	@Test
	public void testAddMass() {
		Mass kStart = MassKt.getLb(5);
		Mass kEnd = kStart.plus(MassKt.getLb(5));
		Logger.log("Calc-ed mass: " + kEnd.getLb()); // the more units the better
		assertEquals(MassKt.getLb(10).getKilogram(), kEnd.getKilogram(), 0.01);
		Mass kMult = kEnd.times(3);
		assertEquals(MassKt.getLb(30).getKilogram(), kMult.getKilogram(), 0.01);
		Logger.log("Calc-ed mass: " + kMult.getLb()); // the more units the better
	}

	@Test
	public void testNegative() {
		Length yes = LengthKt.getInch(-5);
		System.out.println("YEs: " + yes.getInch());
	}

	@Test
	public void testReflection() {
		var old_ = Arrays.asList(
				new Pose2d(LengthKt.getFeet(5.2), LengthKt.getFeet(17.6), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(9.802), LengthKt.getFeet(17.564), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(17.376), LengthKt.getFeet(21.539), Rotation2dKt.getDegree(25.868)),
				new Pose2d(LengthKt.getFeet(21.7), LengthKt.getFeet(18.847), Rotation2dKt.getDegree(-90)),
				new Pose2d(LengthKt.getFeet(21.7), LengthKt.getFeet(16.619), Rotation2dKt.getDegree(-90)));

		for (Pose2d pose : old_)
			System.out.println("start pose: " + Util.toString(pose));

		var new_ = Util.reflectTrajectory(old_);
		for (Pose2d pose : new_)
			System.out.println("reflected pose: " + Util.toString(pose));

	}

}
