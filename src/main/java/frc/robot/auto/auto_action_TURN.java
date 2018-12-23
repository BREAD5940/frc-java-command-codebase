/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;
/**
 * Shifter command to shift to high gear
 */
public class auto_action_TURN extends Command {

  double starting_angle;
  double target_angle;
  String gear;
  double turn_kp;
  double output;
  double max_turn_speed;
  double raw_left;
  double raw_right;
  
  /**
   * Turn a specified number of degrees in a specified gear
   * @param target_angle
   * @param gear "low" or "high"
   */
  public auto_action_TURN(double target_angle, String gear) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    this.target_angle = target_angle;
  }

  // public static final drivetrain drivetrain  = new drivetrain();

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    starting_angle = Robot.gyro.getAngle();
    if (gear == "low") { Robot.drivetrain.setLowGear(); turn_kp = RobotConfig.turn_auto_kp_low; max_turn_speed = RobotConfig.max_turn_speed_low; }
    else if (gear == "high") { Robot.drivetrain.setHighGear(); turn_kp = RobotConfig.turn_auto_kp_high; max_turn_speed = RobotConfig.max_turn_speed_high; }
    else { throw new IllegalArgumentException("Cannot set gear to " + this.gear + "!" ); }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // TODO better PID implamentation, for now sutck with P
    output = Robot.drivetrain.shitty_P_loop(turn_kp, target_angle, Robot.gyro.getAngle(), RobotConfig.max_turn_speed_high, 1); // Get a shitty P loop output
    raw_left = EncoderLib.distanceToRaw(output, RobotConfig.left_wheel_effective_diameter, RobotConfig.POSITION_PULSES_PER_ROTATION);
    raw_right = EncoderLib.distanceToRaw(output, RobotConfig.right_wheel_effective_diameter, RobotConfig.POSITION_PULSES_PER_ROTATION);
    Robot.drivetrain.setLeftSpeedRaw(raw_left);
    Robot.drivetrain.setRightSpeedRaw(raw_right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ( (Math.abs(Robot.gyro.getRate() ) < RobotConfig.turn_auto_angular_velocity_tolerence)
      && (Math.abs(Robot.gyro.getAngle()) < RobotConfig.turn_auto_angle_tolerence)) {
        return true;
      } else { return false; }
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
