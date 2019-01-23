package frc.robot.lib.motion.followers;

import java.io.File;

import frc.math.Pose2d;
import frc.math.Pose2dWithCurvature;
import frc.math.Translation2d;
import frc.math.Util;
import frc.robot.Constants;
import frc.robot.lib.Logger;
import frc.robot.lib.Units;
import frc.robot.lib.motion.DCMotorTransmission;
import frc.robot.lib.motion.DifferentialDrive;
import frc.robot.lib.motion.PathfinderTrajectory;
import frc.robot.lib.motion.TimedTrajectorySegment;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * This class is core to path planning and trajectory following. This will 
 * take a Pathfinder trajectory or CSV file or filepath as an input, make
 * it into something that can be used for pathing, and handle basic questions
 * like "am I there yet?" (answer: no, shut up Jimmy)
 * <p>
 * The trajectory is stored in a PathfinderTrajectory object, which is basically
 * a list of TimedTrajectorySegments.
 * 
 * @author Matthew Morley
 */
public class PathfinderTrajectoryIterator {

  PathfinderTrajectory mTrajectory;

  TimedTrajectorySegment lastSegment = new TimedTrajectorySegment();

  private int progress = 0;

  private boolean trajectoryOutOfBounds = false;

  public PathfinderTrajectoryIterator(Trajectory traj) {
    mTrajectory = new PathfinderTrajectory(traj);
    lastSegment = mTrajectory.getLast();
  }

  public PathfinderTrajectoryIterator(File file) {
    this(Pathfinder.readFromCSV(file));
  }

  public PathfinderTrajectoryIterator(String file) {
    this(new File("/home/lvuser/deploy/paths/" + file + ".pf1.csv"));
  }

  public PathfinderTrajectoryIterator(PathfinderTrajectory traj) {
    mTrajectory = traj;
    lastSegment = traj.getLast();
  }

  public void setTrajectory(PathfinderTrajectory _traj_) {
    mTrajectory = _traj_;
    lastSegment = mTrajectory.getFirst();
  }

  public boolean isDone(Pose2dWithCurvature error) {
    return isDone(error.getPose());
  }

  public boolean isDone(Pose2d error) {
    boolean withinAngleTolerence = (error.getRotation().getRadians() - mTrajectory.getLast().pose().getRotation().getRadians() < Constants.kPathingAngleTolerence );
    boolean withinPositionTolerence = ( error.getTranslation().x() < Constants.kPathingPositionTolerence.x() && 
                                      error.getTranslation().y() < Constants.kPathingPositionTolerence.y() );
    
    //TODO figure out better end behavior than just being ded
    return (progress >= mTrajectory.length - 1 ) || trajectoryOutOfBounds || (withinAngleTolerence && withinPositionTolerence);
  }

  public int getProgress() {
    return progress;
  }

  public TimedTrajectorySegment advance(double mDt) {
    Logger.log("trying to advance the segment");
    if ( progress >= mTrajectory.length  ) {
      Logger.log("not advancing, array out of bounds! returning the last known segment instead"); 
      trajectoryOutOfBounds = true;
      // TODO figure out better end behavior than just being ded
      return lastSegment;
    }
    TimedTrajectorySegment seg;
    if ( mDt < Util.kEpsilon ) {
      seg = lastSegment;
    } else {
      seg = mTrajectory.get(progress);
      progress += 1;
      // return seg;
    }
    Logger.log("segmant iterated!");
    Logger.log("velocity of the segment: " + seg.velocity);
    return seg;
  }

}