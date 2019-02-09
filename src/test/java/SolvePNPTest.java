import java.util.ArrayList;

import org.junit.Test;

import frc.robot.subsystems.VisionProcessor;

public class SolvePNPTest {

	@Test
	public void testPNP() throws InterruptedException {

		ArrayList<double[]> mXCoordinates = new ArrayList<double[]>();//Arrays.asList(
		// m15inX, m24inX, m36inX, m48inX, m60inX
		// ));

		ArrayList<double[]> mYCoordinates = new ArrayList<double[]>();//Arrays.asList(
		// m15inY, m24inY, m36inY, m48inY, m60inY
		// ));

		mXCoordinates.add(new double[]{179, 165, 141, 153});
		mXCoordinates.add(new double[]{177, 167, 146, 156});
		mXCoordinates.add(new double[]{177, 171, 158, 164});
		mXCoordinates.add(new double[]{166, 161, 152, 157});
		mXCoordinates.add(new double[]{158, 163, 159, 156, 153});

		mYCoordinates.add(new double[]{91, 155, 151, 88});
		mYCoordinates.add(new double[]{94, 147, 145, 92});
		mYCoordinates.add(new double[]{100, 134, 134, 99});
		mYCoordinates.add(new double[]{103, 130, 129, 102});
		mYCoordinates.add(new double[]{104, 106, 125, 125, 120});

		VisionProcessor proc = new VisionProcessor(true);

		for (int i = 0; i < mXCoordinates.size() - 1; i++) {

			double[] mCurrentX = mXCoordinates.get(i);
			double[] mCurrentY = mYCoordinates.get(i);

			proc.update(mCurrentX, mCurrentY);
			System.out.println("------------------------------------------");
		}
	}

}
