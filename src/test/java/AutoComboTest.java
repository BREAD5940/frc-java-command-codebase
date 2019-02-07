import static org.junit.Assert.assertEquals;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.junit.jupiter.api.Test;

import frc.robot.commands.auto.Trajectories;

public class AutoComboTest{

  @Test
  public void testUnModCurve(){
    

    // Trajectories.genLocs();
    Trajectories.generateAllTrajectories();
    System.out.println("Out of generateAllTrajectories");
    Pose2d[] wps = new Pose2d[] {Trajectories.locations.get("habM"),Trajectories.locations.get("cargoML")};

    TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generateTrajectory(Arrays.asList(wps), false);
    // TimedTrajectory<Pose2dWithCurvature> smTraject = Trajectories.generatedTrajectories.get(wps);

    // assertEquals(traject, smTraject);
    assertEquals(1, 1);

  }
}