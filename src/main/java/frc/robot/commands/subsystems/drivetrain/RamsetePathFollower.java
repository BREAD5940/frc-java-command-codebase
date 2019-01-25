package frc.robot.commands.subsystems.drivetrain;

import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RamsetePathFollower extends Command {

  TimedTrajectory<Pose2dWithCurvature> trajectory_;

  RamseteTracker tracker_;

  public RamsetePathFollower(TimedTrajectory<Pose2dWithCurvature> trajectory) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    trajectory_ = trajectory;
    tracker_ = Robot.drivetrain.getRamseteTracker();
    tracker_.reset(trajectory_);
  }

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
