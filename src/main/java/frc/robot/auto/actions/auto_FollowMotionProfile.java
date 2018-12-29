/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
// import frc.robot.subsystems.DriveTrain;
import frc.robot.lib.MotionProfileLib;




/**
 * Future pathfinder auto comand. Not functional right now.
 */
public class auto_FollowMotionProfile extends Command {
  String leftProfilePath, rightProfilePath;
  MotionProfileStatus leftMPStatus = Robot.drivetrain.m_left_MP_Status;
  MotionProfileStatus rightMPStatus = Robot.drivetrain.m_right_MP_Status;
  boolean isDone = false;

  // TalonSRX leftTalon = Robot.drivetrain.m_left_talon;

  /**
   * Run a motion profile path given the path to the csv file on the RoboRio
   * @param path to left side csv on roboRio file system
   * @param path to right side csv on roboRio file system
   */
  public auto_FollowMotionProfile(String leftProfilePath, String rightProfilePath) {
    this.leftProfilePath = leftProfilePath;
    this.rightProfilePath = rightProfilePath;
    requires(Robot.drivetrain); // reserve the drivetrain subsystem
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // Process the CSVs and stream to the talons
    MotionProfileLib.motionProfileInit(Robot.drivetrain.m_left_talon, 
      leftProfilePath, RobotConfig.driveTrain.left_wheel_effective_diameter, leftMPStatus);
    MotionProfileLib.motionProfileInit(Robot.drivetrain.m_right_talon, 
      rightProfilePath, RobotConfig.driveTrain.left_wheel_effective_diameter, rightMPStatus);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.drivetrain.m_left_talon.getMotionProfileStatus(leftMPStatus);     // periodic calls to get the status of the Talon SRX
    Robot.drivetrain.m_right_talon.getMotionProfileStatus(rightMPStatus);

    Robot.drivetrain.m_left_talon.processMotionProfileBuffer();     // streams data points from the MPB to the lower-level buffer
    Robot.drivetrain.m_right_talon.processMotionProfileBuffer();

    if (leftMPStatus.btmBufferCnt > 50 || leftMPStatus.topBufferCnt == 0)
      Robot.drivetrain.m_left_talon.set(ControlMode.MotionProfile, 1);

    if (rightMPStatus.btmBufferCnt > 50 || rightMPStatus.topBufferCnt == 0)
      Robot.drivetrain.m_right_talon.set(ControlMode.MotionProfile, 1);

    if (leftMPStatus.isLast) {
      System.out.println("Done with Left Profile");
      Robot.drivetrain.m_left_talon.set(ControlMode.MotionProfile, 0);
      isDone = true;
    }

    if (rightMPStatus.isLast) {
      System.out.println("Done with Right Profile");
      Robot.drivetrain.m_right_talon.set(ControlMode.MotionProfile, 0);
      isDone = true;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.m_left_talon.set(ControlMode.Velocity, 0);
    Robot.drivetrain.m_right_talon.set(ControlMode.Velocity, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drivetrain.m_left_talon.set(ControlMode.Velocity, 0);
    Robot.drivetrain.m_right_talon.set(ControlMode.Velocity, 0);
  }
}
