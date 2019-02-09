
import org.junit.jupiter.api.Test;

import frc.robot.lib.PointFinder;

public class PointFinderTest {

	@Test
	public void testCorners1() {
		double[] xCoords = new double[]{11, 0, 7, 13, 5};
		double[] yCoords = new double[]{9, 10, 10, 0};
		PointFinder finder;
		finder = new PointFinder(xCoords, yCoords);

		System.out.println("Top left corner: " + finder.getTopLeft());
		System.out.println("Top right corner: " + finder.getTopRight());
		System.out.println("Bottom left corner: " + finder.getBottomLeft());
		System.out.println("Bottom right corner: " + finder.getBottomRight());
	}
}
