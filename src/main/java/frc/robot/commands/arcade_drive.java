package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Default drivetrain command. This *should* be called as the default drivetrain command and be overridden in autononmous (provided auto requires drivetrain???)
 * This command uses the Robot.m_oi to set the speed based on xbox controller inputs, arcade style
 * @author Matthew Morley
 */
public class arcade_drive extends Command {
  // System.out.println("im an arcade drive command!");
  /** 
   * This command runs arcade drive as the default command for the drivetrain.
   * This command will reserve the drivetrain.
   */
  public arcade_drive(){
    requires(Robot.drivetrain);
  }

  // drivetrain drivetrain = new drivetrain();

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.arcadeDriveMethod(0, 0);
    System.out.println("arcade drive command init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.drivetrain.arcadeDriveMethod(Robot.m_oi.getForwardAxis(), Robot.m_oi.getTurnAxis());
    Robot.drivetrain.m_left_talon.set(ControlMode.PercentOutput, 1);
    Robot.arcade_running = true;
    System.out.println("arcade drive command execute");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
    // System.out.println("we aint done chief");
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.arcadeDriveMethod(0, 0);
    System.out.println("arcade end called");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drivetrain.arcadeDriveMethod(0, 0);
  }
}
