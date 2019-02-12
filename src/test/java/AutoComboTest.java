import static org.junit.Assert.assertEquals;

import java.io.FileWriter;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.junit.jupiter.api.Test;

import com.opencsv.CSVWriter;

import frc.robot.commands.auto.Trajectories;

public class AutoComboTest {

	@Test
	public void testUnModCurve() {

		Trajectories.generateAllTrajectories(false); // so we aren't real
		System.out.println("Out of generateAllTrajectories");
		Pose2d[] wps = new Pose2d[]{Trajectories.locations.get("habM"), Trajectories.locations.get("cargoML")}; //this should require None Correction

		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generateTrajectoryHighGear(Arrays.asList(wps), false);
		TimedTrajectory<Pose2dWithCurvature> smTraject = Trajectories.generatedHGTrajectories.get("habM to cargoML");
		// System.out.println(Trajectories.generatedTrajectories.get("habM to cargoML"));
		// System.out.println(smTraject);

		System.out.println(traject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet());
		System.out.println(traject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet());
		System.out.println(smTraject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet());
		System.out.println(smTraject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet());

		checkTrajectsEqual(traject, smTraject);

		assertEquals(traject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet(), smTraject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet(), 1);
		assertEquals(traject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet(), smTraject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet(), 1);

	}

	@Test
	public void testModCurveEnds() {
		// Trajectories.generateAllTrajectories(false); // so we aren't real
		System.out.println("Out of generateAllTrajectories");
		Pose2d[] wps = new Pose2d[]{Trajectories.locations.get("habM"), Trajectories.locations.get("cargoML")}; //this should require None Correction
		int count = 0;
		HashMap<String, double[][]> toThrow = new HashMap<String, double[][]>();

		for (String sKey : Trajectories.locations.keySet()) {
			for (String eKey : Trajectories.locations.keySet()) {
				if (!((Trajectories.grabs.contains(sKey) && Trajectories.grabs.contains(eKey)) || (Trajectories.puts.contains(sKey) && Trajectories.puts.contains(eKey)))) {
					TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedHGTrajectories.get(sKey + " to " + eKey);
					System.out.println("Current traject: " + sKey + " to " + eKey);

					double exX = Trajectories.locations.get(sKey).getTranslation().getX().getFeet();
					double exY = Trajectories.locations.get(sKey).getTranslation().getY().getFeet();
					double exxX = Trajectories.locations.get(eKey).getTranslation().getX().getFeet();
					double exxY = Trajectories.locations.get(eKey).getTranslation().getY().getFeet();

					double X = traject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet();
					double Y = traject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet();
					double xX = traject.getPoints().get(traject.getPoints().size() - 1).getState().getPose().getTranslation().getX().getFeet();
					double xY = traject.getPoints().get(traject.getPoints().size() - 1).getState().getPose().getTranslation().getY().getFeet();

					double tol = 0.1;
					if (tol < Math.abs(exX - X) || tol < Math.abs(exY - Y) || tol < Math.abs(exxX - xX) || tol < Math.abs(exxY - xY)) {
						// throw new AssertionFailedError();
						toThrow.put(sKey + " to " + eKey, new double[][]{{exX, exY}, {X, Y}, {exxX, exxY}, {xX, xY}});
						System.out.printf("First error: %f, %f   Second error: %f, %f\n", Math.abs(exX - X), Math.abs(exY - Y), Math.abs(exxX - xX), Math.abs(exxY - xY));
						count++;
					}
				}
			}
		}
		// new Plot(FieldConstraints.pointsAsDoubles(traject.getPoints()), new double[][]{{exX, exY}, {exxX, exxY}});
		try {
			CSVWriter csvWriter = new CSVWriter(new FileWriter("src/test/failures.csv"));
			for (String k : toThrow.keySet()) {
				csvWriter.writeNext(new String[]{k, "first point", "(" + Double.toString(toThrow.get(k)[0][0]) + ", " + Double.toString(toThrow.get(k)[0][1]) + ")",
						"(" + Double.toString(toThrow.get(k)[1][0]) + ", " + Double.toString(toThrow.get(k)[1][1]) + ")"});
				csvWriter.writeNext(new String[]{k, "last point", "(" + Double.toString(toThrow.get(k)[2][0]) + ", " + Double.toString(toThrow.get(k)[2][1]) + ")",
						"(" + Double.toString(toThrow.get(k)[3][0]) + ", " + Double.toString(toThrow.get(k)[3][1]) + ")"});
			}

			csvWriter.close();
		} catch (Exception e) {
			System.out.println("csv is ded");
		}
		System.out.printf("Failed tests: %d\n", count);
	}

	private void checkTrajectsEqual(TimedTrajectory<Pose2dWithCurvature> expected, TimedTrajectory<Pose2dWithCurvature> actual) {
		assertEquals(expected.getReversed(), actual.getReversed());
		List<TimedEntry<Pose2dWithCurvature>> ePoints = expected.getPoints();
		List<TimedEntry<Pose2dWithCurvature>> aPoints = actual.getPoints();

		assertEquals(ePoints.size(), aPoints.size());

		for (int i = 0; i < aPoints.size(); i++) {
			// assertEquals(ePoints.get(i).getT().getSecond(), aPoints.get(i).getT().getSecond(),0.01);
			// assertEquals(ePoints.get(i).getAcceleration().getValue(), aPoints.get(i).getAcceleration().getValue(),0.01);
			assertEquals(ePoints.get(i).getState().getPose().getTranslation().getX().getFeet(), aPoints.get(i).getState().getPose().getTranslation().getX().getFeet(), 1);
			assertEquals(ePoints.get(i).getState().getPose().getTranslation().getY().getFeet(), aPoints.get(i).getState().getPose().getTranslation().getY().getFeet(), 1);
		}

	}
}
