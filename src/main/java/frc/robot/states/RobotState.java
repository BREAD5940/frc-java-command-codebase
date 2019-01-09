package frc.robot.states;

public class RobotState {

  public enum DriveMode {
    AUTO, OPERATOR;
  }

  public DriveMode drivemode = DriveMode.AUTO;

}