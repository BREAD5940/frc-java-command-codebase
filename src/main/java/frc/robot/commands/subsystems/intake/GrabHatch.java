package frc.robot.commands.subsystems.intake;

import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.Robot;

  /**
   * auto_DriveDistance is a basic auto action. It should drive in a straight-ish line, as it uses 
   * nested PID loops to correct for errors caused by differing coefficients of friction. 
   */
public class GrabHatch extends Command {
  double demand, runtime;

  public GrabHatch() {

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Don't have to do anything, just wait for the timeout to trigger
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
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
