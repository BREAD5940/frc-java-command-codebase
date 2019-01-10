package frc.robot.states;

import frc.math.Pose2d;
import frc.robot.RobotConfig;

/**
 * This class uses math inspired (read: on permanent loan) from 254 and based on 
 * work in https://github.com/strasdat/Sophus/tree/master/sophus
 * The state of the robot is based on a Pose2d object, which is a representation
 * of the pose of the robot in translational and rotational elements. 
 */
public class RobotState  {
  
  Pose2d pose_ = new Pose2d();

  public boolean isWithinTolerence() {
    // return (  )
    return true;
  }

}