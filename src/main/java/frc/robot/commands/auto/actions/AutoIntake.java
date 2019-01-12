package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

  /**
   * auto_Intake is a basic auto action. It activates the intake based on an inputted
   * demand and runtime
   */
public class AutoIntake extends Command {
  double demand, runtime;

  /**
   * Run the intake at a set speed for a set number of seconds.
   * To outtake, set the speed to a negative number.
   * @param demand from -1 (outtake) to 1 (intake)
   * @param runtime in seconds
   */
  public AutoIntake(double demand, double runtime) {
    this.demand = demand;
    this.runtime = runtime;
    requires(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.intake.setSpeed(demand);
    setTimeout(runtime); // set the timeout
    System.out.println("auto intake init!");
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
    Robot.intake.setSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.intake.setSpeed(0);
  }
}
