import org.junit.Test;

import frc.math.Util;
import frc.robot.lib.Logger;
import frc.robot.lib.motion.PathfinderTrajectory;
import frc.robot.lib.motion.TimedTrajectorySegment;

import static org.junit.Assert.assertEquals;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.net.URL;
import java.util.Random;
import javax.swing.JFrame;
import info.monitorenter.gui.chart.Chart2D;
import info.monitorenter.gui.chart.ITrace2D;
import info.monitorenter.gui.chart.traces.Trace2DSimple;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;

public class PurePursuitFollowerTest {

  @Test
  public void testCurvature() {

    double dt, position, velocity, acceleration, jerk, heading;

    dt = position = velocity = acceleration = jerk = heading = 0;

    Segment seg0 = new Segment(dt, 0, 0, position, velocity, acceleration, jerk, heading);

    Segment seg1 = new Segment(dt, 0, 2, position, velocity, acceleration, jerk, heading);

    Segment seg2 = new Segment(dt, 0, 5, position, velocity, acceleration, jerk, heading);

    double curvature = TimedTrajectorySegment.curvatureFromPathfinder(seg0, seg1, seg2);
    Logger.log("Curvature: " + curvature);
    assertEquals(0, curvature, 0.01);

    seg0 = new Segment(dt, 0.0f, 0.0f, position, velocity, acceleration, jerk, heading);

    seg1 = new Segment(dt, 2.0f, (2.0f * Math.sqrt(3.0f)), position, velocity, acceleration, jerk, heading);

    seg2 = new Segment(dt, 4.0f, 4.0f, position, velocity, acceleration, jerk, heading);
    
    double distance = TimedTrajectorySegment.distanceFromPathfinder(seg0, seg2);
    Logger.log("Distance: " + distance);
    assertEquals(Math.sqrt(32.0), distance, 0.01);

    curvature = TimedTrajectorySegment.curvatureFromPathfinder(seg0, seg1, seg2);
    Logger.log("Curvature 2: " + curvature);
    assertEquals(0.25f, curvature, 0.01);

    seg0 = new Segment(dt, 0.0f, 0.0f, position, velocity, acceleration, jerk, heading);

    seg1 = new Segment(dt, -2.0f, (2.0f * Math.sqrt(3.0f)), position, velocity, acceleration, jerk, heading);

    seg2 = new Segment(dt, -4.0f, 4.0f, position, velocity, acceleration, jerk, heading);

    curvature = TimedTrajectorySegment.curvatureFromPathfinder(seg0, seg1, seg2);
    Logger.log("Curvature 3: " + curvature);
    assertEquals(-0.25f, curvature, 0.01);


  }


  @Test
  public void testImport() {

  //   Waypoint[] points = new Waypoint[] {
  //     // new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
  //     new Waypoint(0, 0, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
  //     new Waypoint(4, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
  // };
  
  // Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 3, 6, 60.0);
  // Trajectory pfTrajectory = Pathfinder.generate(points, config);
  File csv = new File("./src/main/deploy/paths/test.pf1.csv");
  Trajectory pfTrajectory = Pathfinder.readFromCSV(csv);

  // Logger.log("Reference trajectory ------------------");
  
  // for(int i=0; i<pfTrajectory.length() - 1; i++) {
  //   Segment seg = pfTrajectory.get(i);
  //   Logger.log("Time: " + Util.round(seg.dt * i, 1) + " | X: " + Util.round(seg.x, 2) + " Y: " + Util.round(seg.y, 2) + " Velocity: " + Util.round(seg.velocity, 2));
  // }



  PathfinderTrajectory testTraj = PathfinderTrajectory.readFromTrajectory(pfTrajectory);

  // Logger.log("Converted trajectory ------------------");
  // for(int i=0; i<pfTrajectory.length() - 1; i++) {
  //   TimedTrajectorySegment seg  = testTraj.get(i);
  //   Logger.log("Time: " + Util.round(seg.time, 1) + " | X: " + Util.round(seg.pose().getTranslation().x() , 2) + " Y: " + Util.round(seg.pose().getTranslation().y(), 2) + " Velocity: " + Util.round(seg.velocity, 2));
  // }

  }
  

}