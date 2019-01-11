package frc.robot.lib.purepursuit;

import frc.math.Pose2d;
import frc.math.Translation2d;
import frc.math.Rotation2d;
import frc.math.Twist2d;

import java.util.ArrayList;
import java.util.Collections;

public class PurePursuitTrajectory {

  private ArrayList<PursuitWaypoint> trajectory_;

  /**
   * Construct a trajectory from an ArrayList of waypoints
   * @param waypoints in the trajectory
   */
  public PurePursuitTrajectory(ArrayList<PursuitWaypoint> waypoints) {
    trajectory_ = waypoints;
  }

  /**
   * Return the distance from a pose to a specified index of the trajectory
   * 
   * @param pose2d of the robot
   * @param index of the trajectory
   */
  public double getDistance(Pose2d pose, int index) {
    return new Translation2d(trajectory_.get(index).pose_.getTranslation(), 
            pose.getTranslation()).norm();
  }

  /**
   * Return the index of the closest waypoint to the given pose2d and last known
   * arraylist index
   * @param pose given pose
   * @param lastKnown waypoint index
   * @return int of the closest waypoint
   */
  public int getClosestWaypoint(Pose2d pose, int lastKnown) {
    ArrayList<Double> distances = new ArrayList<Double>();
    for ( int i=lastKnown; i<trajectory_.size(); i++ ) {
      distances.add(i, getDistance(pose, i));
    }
    int minIndex = distances.indexOf(Collections.min(distances));
    return minIndex;
  }


}