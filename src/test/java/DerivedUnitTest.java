import static org.junit.Assert.assertEquals;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.junit.jupiter.api.Test;

import frc.robot.lib.Logger;

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
	public void testDriveDistancePoseOffset() {
		var mCurrentPose = new Pose2d(new Translation2d(), Rotation2dKt.getDegree(45));
		var distance = LengthKt.getFeet(Math.sqrt(2));
		var mDelta = new Pose2d(new Translation2d(distance, mCurrentPose.getRotation()), mCurrentPose.getRotation());
		System.out.println("starting pose: " + mCurrentPose.getTranslation().getX().getFeet() + "," + mCurrentPose.getTranslation().getY().getFeet());
		System.out.println("ending pose: " + mDelta.getTranslation().getX().getFeet() + "," + mDelta.getTranslation().getY().getFeet());

		var expected = new Translation2d(LengthKt.getFeet(1), LengthKt.getFeet(1));
		assertEquals(expected, mDelta.getTranslation());
	}

}
