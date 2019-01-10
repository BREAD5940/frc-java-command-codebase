package frc.robot.states;

import frc.math.Pose2d;
import frc.math.Translation2d;
import frc.robot.RobotConfig;

/**
 * This class uses math inspired (read: on permanent loan) from 254 and based on 
 * work in https://github.com/strasdat/Sophus/tree/master/sophus
 * The state of the robot is based on a Pose2d object, which is a representation
 * of the pose of the robot in translational and rotational elements. 
 * 
 * @author Matthew Morley
 */
public class RobotState  {
  
  private Pose2d pose_ = new Pose2d();

  public Pose2d getPose() {
    return pose_;   
  }

  /**
   * This method checks if the provided target pose of the robot is within the target of the 
   * current pose of the robot. Pass this function a desired pose and it will spit out if it's
   * "close enough" to complete the action
   * <p>
   * @param pose the target pose of the robot
   * @return bool, true if the robot is within tolerence
   */
  public boolean isWithinTolerence(Pose2d pose) {
    return (new Translation2d(pose_.getTranslation(), pose.getTranslation()).norm() < RobotConfig.auto.tolerences.position_tolerence) ;
  }

}