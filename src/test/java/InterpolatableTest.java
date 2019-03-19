import static org.junit.Assert.assertEquals;

import java.util.TreeMap;

import org.junit.Test;

import frc.robot.lib.InterpolatableLut;
import frc.robot.lib.InterpolatableLutEntry;

public class InterpolatableTest {

	@Test
	public void testInterpolate() {
		InterpolatableLutEntry entry1 = new InterpolatableLutEntry(Double.valueOf(0));

		InterpolatableLutEntry entry2 = new InterpolatableLutEntry(Double.valueOf(20));

		InterpolatableLutEntry entry3 = new InterpolatableLutEntry(Double.valueOf(60));

		TreeMap<Double, InterpolatableLutEntry> map = new TreeMap<>();

		map.put(Double.valueOf(0), entry1);
		map.put(Double.valueOf(10), entry2);
		map.put(Double.valueOf(20), entry3);

		var interpolatingTable = new InterpolatableLut(map);

		// System.out.println("Interpolated with key 5: " + interpolatingTable.interpolate(Double.valueOf(5)));

		assertEquals(10, interpolatingTable.interpolate(Double.valueOf(5)), 0.1);

		// System.out.println("Interpolated with key 10: " + interpolatingTable.interpolate(Double.valueOf(10)));

		assertEquals(20, interpolatingTable.interpolate(Double.valueOf(10)), 0.1);

		// System.out.println("Interpolated with key 20: " + interpolatingTable.interpolate(Double.valueOf(20)));

		assertEquals(60, interpolatingTable.interpolate(Double.valueOf(20)), 0.1);

    assertEquals(40, interpolatingTable.interpolate(Double.valueOf(15)), 0.1);

    
    assertEquals(60, interpolatingTable.interpolate(Double.valueOf(100000)), 0.1);


		// assertTrue(() -> {
		// 	return false;
		// });

	}

}
