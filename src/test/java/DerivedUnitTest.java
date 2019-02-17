import static org.junit.Assert.assertEquals;

import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
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

}
