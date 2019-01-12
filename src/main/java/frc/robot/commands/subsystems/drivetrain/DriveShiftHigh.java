package frc.robot.commands.subsystems.drivetrain;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Shifter command to shift to high gear
 */
public class DriveShiftHigh extends Command {
  public DriveShiftHigh() {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.drivetrain); // I don't think it's strictly necessary to
    // require the drivetrain for an instant action
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.setHighGear();
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
