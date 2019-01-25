package frc.robot.commands.auto.pathing;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TrajectoryTrackerCommand extends Command {

  
  private boolean trajectoryFinished = false;

  public TrajectoryTrackerCommand() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // trajectoryTracker.reset(trajectorySource())
    // trajectoryFinished = false
    // LiveDashboard.isFollowingPath = true
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
