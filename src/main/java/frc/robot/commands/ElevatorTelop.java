package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Default command run by the elevator. Sets the elevator height to a fixed setpoint throttle style.
 * For use in teleop.
 * 
 * @author Matthew Morley
 */
public class ElevatorTelop extends Command {

  double targetHeight;

  /** 
   * Requires elevator subsystem
   */
  public ElevatorTelop(){
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    // TODO add logging
    System.out.println("Elevator telop init!"); 
  }

  /**
   * Set the target height of the elevator in inches to the current throttle height
   */
  @Override
  protected void execute() {
    targetHeight = (Robot.elevator.iterativeSetHeight(Robot.m_oi.getElevatorAxis() * 1 ));
    Robot.elevator.setHeight(targetHeight);//targetHeight);
    System.out.println("targetHeight: " + targetHeight + " Elevator axis: " 
      + Robot.m_oi.getElevatorAxis() * 1 + " Get elevator height inches: " + Robot.elevator.getHeight() );
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
