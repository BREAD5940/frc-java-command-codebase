package frc.robot.states;

import frc.math.Pose2d;
import frc.math.Rotation2d;

public class RobotState {

  public enum DriveMode {
    AUTO, OPERATOR;
  }

  public DriveMode drivemode = DriveMode.AUTO;

  private Rotation2d m_rotate = new Rotation2d(0, 0, false);
  private Pose2d m_robotPose = new Pose2d(0, 0, m_rotate);


  public void updatePose() {
    // m_robotPose.transformBy(other)
  }

  public Pose2d getPose() {
    return m_robotPose;
  }

}