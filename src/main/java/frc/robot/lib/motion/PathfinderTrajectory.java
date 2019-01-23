package frc.robot.lib.motion;

import java.io.File;
import java.util.ArrayList;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * This class stores trajectory data, similar to Pathfinder. One *important*
 * distinction is that while for Pathfinder X is forward, for this Y is forward.
 * This object stores everything Pathfinder saves to a CSV file in addition to
 * curvature, if for some reason you wanted that. Which you don't, because pure
 * pursuit sucks.
 * 
 * @author Matthew Morley
 */
public class PathfinderTrajectory {

  ArrayList<TimedTrajectorySegment> mTrajectory;

  public final int length;

  public PathfinderTrajectory(ArrayList<TimedTrajectorySegment> trajectory) {
    mTrajectory = trajectory;
    length = trajectory.size();
  }

  public PathfinderTrajectory(Trajectory trajectory) {
    mTrajectory = readFromTrajectory(trajectory);
    length = mTrajectory.size();
  }

  public PathfinderTrajectory(String filepath) {
    mTrajectory = readFromTrajectory( Pathfinder.readFromCSV(new File("/home/lvuser/deploy/paths/" + filepath + ".pf1.csv")));
    length = mTrajectory.size();
  }

  /**
   * Loop through a provided Trajectory to extract parameters like
   * Y and X position (note: REVERSED relative to pathfinder!), angle
   * in radians, distance along the path and curvature (which is
   * coming soon^tm, but maybe never because pure pursuit sucks)
   */
  public ArrayList<TimedTrajectorySegment> readFromTrajectory(Trajectory trajectory) {
    ArrayList<TimedTrajectorySegment> path = new ArrayList<TimedTrajectorySegment>();

    // Loop thru all the points in the trajectory and convert them to TimedTrajectorySegments
    for( int i=0; i < trajectory.length() - 1; i++ ) {
      Segment seg_ = trajectory.get(i);
      TimedTrajectorySegment segment = TimedTrajectorySegment.fromPathfinderSegment(seg_.dt * i + seg_.dt, seg_ );
      path.add(segment);
    }
    
    return path;
  }

  public TimedTrajectorySegment get(int index) {
    return mTrajectory.get(index);
  }

  public TimedTrajectorySegment getFirst() {
    return mTrajectory.get(0);
  }

  public TimedTrajectorySegment getLast() {
    return mTrajectory.get(mTrajectory.size());
  }

}