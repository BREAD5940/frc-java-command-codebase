import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.TreeMap;

import org.junit.Test;

import frc.robot.lib.InterpolatableLut;
import frc.robot.lib.InterpolatableLutEntry;

public class InterpolatableTest {

	@Test
	public void testInterpolate() {
		InterpolatableLutEntry entry1 = new InterpolatableLutEntry(Double.valueOf(0));

		InterpolatableLutEntry entry2 = new InterpolatableLutEntry(Double.valueOf(10));

		TreeMap<Double, InterpolatableLutEntry> map = new TreeMap<>();

		map.put(Double.valueOf(0), entry1);
		map.put(Double.valueOf(10), entry2);

		var interpolatingTable = new InterpolatableLut(map);

		System.out.println("Interpolated with key 5: " + interpolatingTable.interpolate(Double.valueOf(5)));

		assertTrue(() -> {
			return false;
		});

	}

}
