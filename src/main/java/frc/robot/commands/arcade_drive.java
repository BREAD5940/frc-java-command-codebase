package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.robotconfig;
import frc.robot.subsystems.drivetrain;

/**
 * Default drivetrain command. This *should* be called as the default drivetrain command and be overridden in autononmous (provided auto requires drivetrain???)
 * This command uses the Robot.m_oi to set the speed based on xbox controller inputs, arcade style
 * @author Matthew Morley
 */
public class arcade_drive extends Command {

  /** 
   * This command runs arcade drive as the default command for the drivetrain.
   * This command will reserve the drivetrain.
   */
  public arcade_drive(){
    requires(Robot.drivetrain);
  }

  drivetrain drivetrain = new drivetrain();

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
