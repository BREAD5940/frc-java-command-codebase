package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.command.Command;

public abstract class CommandOnCondition extends Command {

  /**
   * (Maybe) run a command on a condition
   */
  public CommandOnCondition() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  public boolean readyForExecute() {
    return false;
  }

  public void executeOnCondition() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (readyForExecute()) {
      executeOnCondition();
    }
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