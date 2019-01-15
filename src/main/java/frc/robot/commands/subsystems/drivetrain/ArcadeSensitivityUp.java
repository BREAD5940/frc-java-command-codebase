package frc.robot.commands.subsystems.drivetrain;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Sensitivity command to make joystick more responsive
 */
public class ArcadeSensitivityUp extends Command {
  public ArcadeSensitivityUp() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arcade_drive_sensitivity += 0.1
    if (Robot.arcade_drive_sensitivity > 1.5) {
      Robot.arcade_drive_sensitivity = 1.5;
    }
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
