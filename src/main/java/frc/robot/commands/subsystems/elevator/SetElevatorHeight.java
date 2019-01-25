package frc.robot.commands.subsystems.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPresets;

/**
 * Run the elevator to a set height during autonomous using one of the
 * constructors lol. When I wrote this I just realized you could have multiple
 * constructors, so you could have a bunch of different parameters, so I went
 * crazy with mixtures of demand, isInstant, and timeout.
 */
public class SetElevatorHeight extends Command {

  double demand;
  boolean isInstant;
  double timeout = 10; // default dimeout to 10s
  ElevatorPresets heightEnum;

  private enum HeightMode {
    INCHES, PRESET;
  }

  HeightMode heightmode;

  /**
   * Run the elevator to a set height during autonomous. Also set boolean flag for
   * if this command runs instantly or not. Timeout defaults to 10 seconds.
   * 
   * @param demand    in inches
   * @param isInstant flag
   */
  public SetElevatorHeight(double demand, boolean isInstant) {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.elevator);
    this.demand = demand;
    this.isInstant = isInstant;
    this.heightmode = HeightMode.INCHES;
  }

  /**
   * This comstructor defaults to waiting for the elevator to reach the target
   * height set in inches. Timeout will default to 10 seconds
   * 
   * @param demand in inches
   */
  public SetElevatorHeight(double demand) {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.elevator);
    this.demand = demand;
    this.isInstant = false;
    this.heightmode = HeightMode.INCHES;
  }

  /**
   * Run the elevator to a set height during autonomous. Also set boolean flag for
   * if this command runs instantly or not. Timeout will *not* default and must be
   * set in arguments.
   * 
   * @param demand    in inches
   * @param isInstant flag
   * @param timeout   of this command in seconds
   */
  public SetElevatorHeight(double demand, boolean isInstant, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.elevator);
    this.demand = demand;
    this.isInstant = isInstant;
    this.timeout = timeout;
    this.heightmode = HeightMode.INCHES;
  }

  public SetElevatorHeight(ElevatorPresets height, boolean isInstant) {
    // requires(Robot.elevator);
    this.heightEnum = height;
    this.isInstant = isInstant;
    this.heightmode = HeightMode.PRESET;
  }

  @Override
  protected void initialize() {
    switch (heightmode) {
    case INCHES:
      break;
    case PRESET:
      demand = Elevator.getHeightEnumValue(heightEnum);
    default:
      break;
    }
    // Robot.elevator.setHeight(demand);
    setTimeout(timeout);
  }

  @Override
  protected void execute() {
  }

  /**
   * Return boolean of if the elevator is within 0.5 inches of the set height OR
   * the command is supposed to run instantly.
   */
  @Override
  protected boolean isFinished() {
    // return isInstant || Robot.elevator.isWithinTolerence(demand);
    return true;
    
    // if ((isInstant) || (Math.abs(Robot.elevator.getHeight() - demand) < 0.5) || isTimedOut()) {
    //   return true;
    // } else {
    //   return false;
    // }
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
