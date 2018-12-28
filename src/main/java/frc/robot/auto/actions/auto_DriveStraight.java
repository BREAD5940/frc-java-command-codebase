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
   * Drive straight ahead. The target angle is set based on the current robot angle.
   * This uses a position PID loop to set drivetrain speeds. The left and right speeds
   * are manipulated based on a turn PID loop such that the angle remains constant.
   * // TODO determine how the difference in encoder positions if the angle is changed will affect pid
   * @param distance in feet
   */
public class auto_DriveStraight extends Command {
  double distance;
  double actionMaxSpeed;
  double timeout = 15;
  double start_left_distance = Robot.drivetrain.getLeftDistance();
  double start_right_distance = Robot.drivetrain.getRightDistance();
  double end_distance_left;
  double start_gyro_angle;
  double target_gyro_angle;
  double current_angle;
  double forward_speed;
  double turn_speed;
  double angle_error;
  double left_speed_raw, right_speed_raw;

  private ShittyPID forwardPID = new ShittyPID(
    RobotConfig.drive_straight.turn_kp, 
    RobotConfig.drive_auto_forward_velocity_min,
    RobotConfig.drive_auto_forward_velocity_max 
  );

  private ShittyPID turnPID = new ShittyPID(
    RobotConfig.drive_straight.turn_kp, 
    RobotConfig.drive_straight.turn_ki, 
    RobotConfig.drive_straight.minimum_turn_speed, 
    RobotConfig.drive_straight.maximum_turn_speed, 
    RobotConfig.drive_straight.turn_izone, 
    RobotConfig.drive_straight.turn_integral_max
  );
  

  /**
   * auto_DriveStraight drives in a straight line. The target angle is the same angle as the gyro
   * is initilized with. Timeout is 15 seconds.
   * @param distance in feet
   * @param speed maximum speed in feet per second
   */
  public auto_DriveStraight(double distance, double speed, double timeout) {
    this.distance = distance;
    this.actionMaxSpeed = speed;
    this.target_gyro_angle = Robot.gyro.getAngle(); // TODO make sure that the angle is set correctly.
    requires(Robot.drivetrain);
  }

  /**
   * Auto_DriveStraight drives in a straight line. Target angle is the angle at which the action
   * is init'ed at. Speed is default auto speed, and timeout is 15 seconds.
   * @param distance in feet
   */
  public auto_DriveStraight(double distance) {
    this.distance = distance;
    this.actionMaxSpeed = RobotConfig.drive_auto_forward_velocity_max;
    this.target_gyro_angle = Robot.gyro.getAngle(); // TODO make sure that the angle is set correctly on constructor call.
    requires(Robot.drivetrain);
  }

  /**
   * Auto_DriveStraight drives in a straight line. Target angle is the angle at which the action
   * is init'ed at. Speed is default auto speed, and timeout is 15 seconds.
   * @param distance in feet
   * @param angle absolute angle in degrees from auto init
   * 
   * <p> MAKE SURE THAT YOU USE AN ABSOLUTE ANGLE WITH THIS CONSTRUCTOR!
   */
  public auto_DriveStraight(double distance, double angle) {
    this.distance = distance;
    this.actionMaxSpeed = RobotConfig.drive_auto_forward_velocity_max;
    this.target_gyro_angle = angle; // TODO make sure that the angle is set correctly.
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    current_angle = Robot.gyro.getAngle();
    end_distance_left = start_left_distance + distance;
    setTimeout(timeout); // set the timeout
    forwardPID.setSetpoint(end_distance_left);
    turnPID.setSetpoint(target_gyro_angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    forward_speed = forwardPID.update(Robot.drivetrain.getLeftDistance());
    turn_speed = 1 + turnPID.update(Robot.gyro.getAngle());

    left_speed_raw = EncoderLib.distanceToRaw(forward_speed + turn_speed, RobotConfig.left_wheel_effective_diameter / 12, 
      RobotConfig.POSITION_PULSES_PER_ROTATION) / 10;
    right_speed_raw = EncoderLib.distanceToRaw(forward_speed - turn_speed, RobotConfig.right_wheel_effective_diameter / 12, 
      RobotConfig.POSITION_PULSES_PER_ROTATION) / 10;

    Robot.drivetrain.setSpeeds(left_speed_raw, right_speed_raw);

    System.out.println("FORWARD PID: Setpoint: " + forwardPID.getSetpoint() + " Measured: " + Robot.drivetrain.getLeftDistance() + 
      " Error: " + forwardPID.getError() + " OUTPUT VELOCITY (ft/s): " + forwardPID.getOutput());
    System.out.println("TURN PID: Setpoint: " + turnPID.getSetpoint() + " Measured: " + Robot.gyro.getAngle() + 
      " Error: " + turnPID.getError() + " OUTPUT VELOCITY (ft/s): " + turnPID.getOutput());

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ( ((Math.abs(Robot.drivetrain.getRightDistance() - this.distance) < RobotConfig.drive_auto_position_tolerence) 
      // && (Math.abs(Robot.drivetrain.getLeftDistance() - this.distance) < RobotConfig.drive_auto_position_tolerence) 
      && (Math.abs(Robot.drivetrain.getLeftVelocity()) < RobotConfig.drive_auto_velocity_tolerence) 
      && (Math.abs(Robot.drivetrain.getRightVelocity()) < RobotConfig.drive_auto_position_tolerence)
      && (Math.abs(target_gyro_angle - current_angle) < RobotConfig.drive_auto_straight_angle_tolerence ))
      || (isTimedOut()) 
    ){ return true; }
    else { return false; }
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
