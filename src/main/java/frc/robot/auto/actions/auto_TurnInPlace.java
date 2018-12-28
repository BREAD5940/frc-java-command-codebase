/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;
import frc.robot.lib.ShittyPID;

/**
 * Literally just pivot in place by a desired amount
 */
public class auto_TurnInPlace extends Command {

  double starting_angle;
  double target_angle_relative;
  double target_angle;
  double target_angle_absolute;
  boolean isAbsolute = true;
  double output;
  double max_turn_speed;
  double raw_left;
  double raw_right;

  ShittyPID turnPID = new ShittyPID(RobotConfig.auto.auto_turnInPlace.kp, RobotConfig.auto.auto_turnInPlace.ki, 
    RobotConfig.auto.auto_turnInPlace.min_turn_speed, 
    RobotConfig.auto.auto_turnInPlace.max_turn_speed, 
    RobotConfig.auto.auto_turnInPlace.integral_zone, 
    RobotConfig.auto.auto_turnInPlace.max_integral);
  
  /**
   * Turn a specified number of degrees in the default auto gear.
   * This constructor will default to taking the absolute angle
   * to turn to, rather than a relative angle. If you want to 
   * specify, use a bool as the second argument to specify if the
   * angle should be interpreted as absolute or not.
   * @param target_angle
   */
  public auto_TurnInPlace(double target_angle) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    this.target_angle = target_angle;
  }

  /**
   * Turn a specified number of degrees in the default auto gear. 
   * The angle passed is an absolute angle relative to the 
   * angle upon autonomous init.
   * @param target_angle
   */
  public auto_TurnInPlace(double target_angle, boolean isAbsolute) {
    this.isAbsolute = isAbsolute;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    this.target_angle = target_angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    starting_angle = Robot.gyro.getAngle();
    turnPID.setSetpoint(target_angle);


    // If the angle is relative (which it should not be), setup target angle.
    // Otherwise the angle is absolute (relative to auto init) so we don't care.
    if (!(isAbsolute)){ // if isAbsolute is false, and we want a relative angle
      target_angle = target_angle + starting_angle;
    }

    turnPID.setSetpoint(target_angle);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    output = turnPID.update(Robot.gyro.getAngle());
    raw_left = EncoderLib.distanceToRaw(output, RobotConfig.driveTrain.left_wheel_effective_diameter, RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);
    raw_right = (-1) * EncoderLib.distanceToRaw(output, RobotConfig.driveTrain.right_wheel_effective_diameter, RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);
    Robot.drivetrain.setSpeeds(raw_left, raw_right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if ( (Math.abs(Robot.gyro.getRate() ) < RobotConfig.turn_auto_angular_velocity_tolerence)
    //   && (Math.abs(Robot.gyro.getAngle()) < RobotConfig.turn_auto_angle_tolerence)) {
    //     return true;
    //   } else { return false; }

    // TODO so this is how a return works
    return ( (Math.abs(Robot.gyro.getRate() ) < RobotConfig.auto.tolerences.angular_velocity_tolerence)
      && (Math.abs(Robot.gyro.getAngle()) < RobotConfig.auto.tolerences.angle_tolerence));

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
