/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain;
import frc.robot.robotconfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;



/**
 * Shifter command to shift to high gear
 */
public class drive_auto_action extends Command {
  public drive_auto_action() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
  }

  public static final drivetrain drivetrain  = new drivetrain();

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drivetrain.m_left_talon.config_kP(0, robotconfig.m_left_velocity_kp_high, 0);
    drivetrain.m_left_talon.config_kI(0, robotconfig.m_left_velocity_ki_high, 0);
    drivetrain.m_left_talon.config_kD(0, robotconfig.m_left_velocity_kd_high, 0);
    drivetrain.m_left_talon.config_kF(0, robotconfig.m_left_velocity_kf_high, 0);
    drivetrain.m_left_talon.config_IntegralZone(0, robotconfig.m_left_velocity_izone_high, 0);
    drivetrain.m_left_talon.configMaxIntegralAccumulator(0, robotconfig.m_left_velocity_max_integral_high, 0);

    drivetrain.m_right_talon.config_kP(0, robotconfig.m_right_velocity_kp_high, 0);
    drivetrain.m_right_talon.config_kI(0, robotconfig.m_right_velocity_ki_high, 0);
    drivetrain.m_right_talon.config_kD(0, robotconfig.m_right_velocity_kd_high, 0);
    drivetrain.m_right_talon.config_kF(0, robotconfig.m_right_velocity_kf_high, 0);
    drivetrain.m_right_talon.config_IntegralZone(0, robotconfig.m_right_velocity_izone_high, 0);
    drivetrain.m_right_talon.configMaxIntegralAccumulator(0, robotconfig.m_right_velocity_max_integral_high, 0);
    
    drivetrain.shifter_solenoid.set(DoubleSolenoid.Value.kReverse);
    // TODO verify that kForward is high gear

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
