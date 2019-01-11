package frc.robot.lib;

import frc.math.Pose2d;
import frc.robot.lib.PursuitWaypoint;

import java.util.ArrayList;

public class PurePursuitController {
  protected final ArrayList<PursuitWaypoint> waypoints_;
  private double lookahead_;
  private double goalTolerence_;
  private int mLastKnownPosition = 0;

  public PurePursuitController(ArrayList<PursuitWaypoint> waypoints, double lookahead, double goalTolerence ) {
    waypoints_ = waypoints;
    lookahead_ = lookahead;
    goalTolerence_ = goalTolerence;
  }

  /**
   * Return the closest PursuitWaypoint to the robotPose
   * @param robotPose
   * @return the closest PursuitWaypoint
   */
  public PursuitWaypoint getClosestPoint(Pose2d robotPose) {
    return null;
    // TODO make this do stuff
  }

}