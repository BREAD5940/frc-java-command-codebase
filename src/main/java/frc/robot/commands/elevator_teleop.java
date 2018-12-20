package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Default command run by the elevator. Sets the elevator height to a fixed setpoint throttle style.
 * For use in teleop.
 * 
 * @author Matthew Morley
 */
public class elevator_teleop extends Command {

  /** 
   * Requires elevator subsystem
   */
  public elevator_teleop(){
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    // TODO add logging
  }

  /**
   * Set the target height of the elevator in inches to the current throttle height
   */
  @Override
  protected void execute() {
    Robot.elevator.setHeight(Robot.elevator.getElevatorAxisInches());
  }

  @Override
  protected boolean isFinished() {
    return false; // This command never cancels itself
  }

  // TODO decide if the elevator should set itself to 0 on command end
  @Override
  protected void end() {
    Robot.elevator.setHeight(Robot.elevator.getElevatorAxisInches());
  }

  // TODO decide if the elevator should set itself to 0 on command end
  @Override
  protected void interrupted() {
    Robot.elevator.setHeight(Robot.elevator.getElevatorAxisInches());
  }
}
