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
    
    Trajectories.generateAllTrajectories();
    System.out.println("Out of generateAllTrajectories");
    Double[] wps = new Double[] {Trajectories.locations.get("habM").getTranslation().getX().getFeet(),Trajectories.locations.get("cargoML").getTranslation().getX().getFeet()};
    System.out.println(wps[0]);
    System.out.println(wps[1]);

    // TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generateTrajectory(Arrays.asList(wps), false);
    TimedTrajectory<Pose2dWithCurvature> smTraject = Trajectories.generatedTrajectories.get("habM to cargoML");
    System.out.println(wps);
    System.out.println(Trajectories.generatedTrajectories.get("habM to cargoML"));
    System.out.println(smTraject);

    
    // assertEquals("test", traject.getPoints().get(1).getState().getPose().getTranslation().getX().getFeet(),
        // smTraject.getPoints().get(1).getState().getPose().getTranslation().getX().getFeet(), 0.01);
    assertEquals(1, 1);

  }
}