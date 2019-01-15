package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.motion.followers.PurePursuitFollower;

public class PurePursuitPathCommand extends Command {

  private PurePursuitFollower mFollower;

  public PurePursuitPathCommand() {
    mFollower = new PurePursuitFollower();
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mFollower.initPath();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mFollower.update();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mFollower.isDone();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.setPowers(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
