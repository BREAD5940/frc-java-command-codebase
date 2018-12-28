/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;
import frc.robot.lib.ShittyPID;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.Command;


/**
 * Follow a vision target tracked by a limelight
 */
public class FollowVisionTarget extends Command {
  double timeout, targetSpeed, targetPercentOfFrame, angleDeltaX, angleDeltaY, forwardSpeed, turnSpeed, leftSpeedRaw, rightSpeedRaw;
  boolean followRange = false;

  private ShittyPID forwardPID = new ShittyPID(
    RobotConfig.followVisionTarget.forward.kp,
    RobotConfig.drive_auto_forward_velocity_min,
    RobotConfig.drive_auto_forward_velocity_max 
  );
  private ShittyPID turnPID = new ShittyPID(
    RobotConfig.followVisionTarget.turn.kp,
    RobotConfig.followVisionTarget.turn.ki,
    RobotConfig.followVisionTarget.turn.min_turn_speed,
    RobotConfig.followVisionTarget.turn.max_turn_speed, 
    RobotConfig.followVisionTarget.turn.integral_zone, 
    RobotConfig.followVisionTarget.turn.max_integral
  );
  
  /**
   * Follow a limelight vision target. Move toward the target at the set speed 
   * and timeout after a set time. Tracks only angle, not range!
   * @param speed
   * @param timeout
   */
  public FollowVisionTarget(double speed, double timeout) {
    this.timeout = timeout;
    this.targetSpeed = speed;
    requires(Robot.drivetrain);
  }

  /**
   * Follow a limelight vision target. Track both range and angle
   * of the target to the limelight
   * @param speed to track forwards at
   * @param targetPercentOfFrame percent of frame taken up by target
   * @param timeout
   */
  public FollowVisionTarget(double speed, double targetPercentOfFrame, double timeout) {
    this.timeout = timeout;
    this.targetSpeed = speed;
    this.targetPercentOfFrame = targetPercentOfFrame;
    this.followRange = true;
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(timeout);
    forwardPID.setMaxOutput(targetSpeed);
    if (followRange){ 
      forwardPID.setSetpoint(targetPercentOfFrame);
      forwardPID.setKpGain(RobotConfig.followVisionTarget.forward.kp_rangeMode);
    } 
    else { forwardPID.setSetpoint(targetSpeed); }
    turnPID.setSetpoint(0);
    angleDeltaX = Robot.limelight.getDx();
    targetPercentOfFrame = Robot.limelight.getTargetArea();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if ( Robot.limelight.getTrackedTargets() != 0 ) {
      if (followRange) {
        forwardSpeed = forwardPID.update(Robot.limelight.getTargetArea());
      } else {
        forwardSpeed = forwardPID.update(Robot.drivetrain.getLeftVelocity());
      }
      turnSpeed = turnPID.update(Robot.limelight.getDxAngle());
      
      leftSpeedRaw = EncoderLib.distanceToRaw(forwardSpeed + turnSpeed, RobotConfig.left_wheel_effective_diameter / 12, 
      RobotConfig.POSITION_PULSES_PER_ROTATION) / 10;
      rightSpeedRaw = EncoderLib.distanceToRaw(forwardSpeed - turnSpeed, RobotConfig.right_wheel_effective_diameter / 12, 
      RobotConfig.POSITION_PULSES_PER_ROTATION) / 10;

      System.out.println("FORWARD PID: Setpoint: " + forwardPID.getSetpoint() + " Measured: " + Robot.drivetrain.getLeftDistance() + 
        " Error: " + forwardPID.getError() + " OUTPUT VELOCITY (ft/s): " + forwardPID.getOutput());
      System.out.println("TURN PID: Setpoint: " + turnPID.getSetpoint()+ 
        " Error: " + turnPID.getError() + " OUTPUT VELOCITY (ft/s): " + turnPID.getOutput());
    } else {
      System.out.println("No targets currently being tracked! Can't track thin air, can I?");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
