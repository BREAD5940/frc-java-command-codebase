import static org.junit.Assert.assertEquals;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.junit.jupiter.api.Test;

import frc.robot.commands.auto.Trajectories;

public class AutoComboTest {

	@Test
	public void testUnModCurve() {

		Trajectories.generateAllTrajectories(false); // so we aren't real
		System.out.println("Out of generateAllTrajectories");
		Pose2d[] wps = new Pose2d[]{Trajectories.locations.get("habM"), Trajectories.locations.get("cargoML")}; //this should require None Correction

		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generateTrajectory(Arrays.asList(wps), false);
		TimedTrajectory<Pose2dWithCurvature> smTraject = Trajectories.generatedTrajectories.get("habM to cargoML");
		// System.out.println(Trajectories.generatedTrajectories.get("habM to cargoML"));
		// System.out.println(smTraject);

		System.out.println(traject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet());
		System.out.println(traject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet());
		System.out.println(smTraject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet());
		System.out.println(smTraject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet());

		// checkTrajectsEqual(traject, smTraject);

		assertEquals(traject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet(), smTraject.getPoints().get(0).getState().getPose().getTranslation().getX().getFeet(), 1);
		assertEquals(traject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet(), smTraject.getPoints().get(0).getState().getPose().getTranslation().getY().getFeet(), 1);

	}

	private void checkTrajectsEqual(TimedTrajectory<Pose2dWithCurvature> expected, TimedTrajectory<Pose2dWithCurvature> actual) {
		assertEquals(expected.getReversed(), actual.getReversed());
		List<TimedEntry<Pose2dWithCurvature>> ePoints = expected.getPoints();
		List<TimedEntry<Pose2dWithCurvature>> aPoints = actual.getPoints();

		for (int i = 0; i < ePoints.size(); i++) {
			// assertEquals(ePoints.get(i).getT().getSecond(), aPoints.get(i).getT().getSecond(),0.01);
			// assertEquals(ePoints.get(i).getAcceleration().getValue(), aPoints.get(i).getAcceleration().getValue(),0.01);
			assertEquals(ePoints.get(i).getState().getPose().getTranslation().getX().getFeet(), aPoints.get(i).getState().getPose().getTranslation().getX().getFeet(), 1);
			assertEquals(ePoints.get(i).getState().getPose().getTranslation().getY().getFeet(), aPoints.get(i).getState().getPose().getTranslation().getY().getFeet(), 1);
		}

	}
}
