package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;

public class WaitForElevator extends Command {

  double demand;

  public WaitForElevator(double demand) {
    // Doesn't require anything, just waiting for elevator height
    this.demand = demand;
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
    // return Robot.elevator.isWithinTolerence(demand);
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
