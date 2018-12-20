package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.robotconfig;
import frc.robot.subsystems.drivetrain;

public class arcade_drive extends Command {

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
    double forwardspeed = Robot.m_oi.getForwardAxis() * -1;
    double turnspeed = Robot.m_oi.getTurnAxis();

    if ((forwardspeed < 0.02) && (forwardspeed > -0.02)) { forwardspeed = 0; }
    if ((turnspeed < 0.01) && (turnspeed > -0.01)) { turnspeed = 0; }
    
    if (robotconfig.driving_squared) {
    forwardspeed = forwardspeed * Math.abs(forwardspeed);
    turnspeed = turnspeed * Math.abs(turnspeed);
    }
    if (Robot.drivetrain.current_gear == "high"){
        forwardspeed = forwardspeed * robotconfig.max_forward_speed_high;
        turnspeed = turnspeed * robotconfig.max_turn_speed_high;}
    if (Robot.drivetrain.current_gear == "low"){
        forwardspeed = forwardspeed * robotconfig.max_forward_speed_low;
        turnspeed = turnspeed * robotconfig.max_turn_speed_low;}

    double leftspeed = -forwardspeed + turnspeed;
    double rightspeed = -forwardspeed - turnspeed;

    // drive.setVelocityLeft(1 * 100);
    // drive.setVelocityRight(1 * 100);

    // Robot.drivetrain.m_left_talon.set(ControlMode.Velocity, 1024);
    // Robot.drivetrain.m_right_talon.set(ControlMode.Velocity, 1024);

    drivetrain.m_left_talon.set(ControlMode.PercentOutput, 0.3);
    drivetrain.m_right_talon.set(ControlMode.PercentOutput, 0.3);
    Robot.arcade_running = true;
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
