package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;
import frc.robot.lib.TerriblePID;

/**
 * Literally just pivot in place by a desired amount
 */
public class TurnInPlace extends Command {

  double starting_angle;
  double target_angle_relative;
  double target_angle;
  double target_angle_absolute;
  boolean isAbsolute = false;
  double output;
  double max_turn_speed;
  double raw_left;
  double raw_right;

  // TerriblePID turnPID = new TerriblePID(RobotConfig.auto.TurnInPlace.kp, RobotConfig.auto.TurnInPlace.ki, 
  //   RobotConfig.auto.TurnInPlace.min_turn_speed, 
  //   RobotConfig.auto.TurnInPlace.max_turn_speed, 
  //   RobotConfig.auto.TurnInPlace.integral_zone, 
  //   RobotConfig.auto.TurnInPlace.max_integral);
  
  double kp = 0.015;
  double ki = 0.0; 
  double kd = 0.1;
  double integralZone = 1000;
  double maxIntegralAccum =1000;

  // public TerriblePID(double kp, double ki, double minOutput, double maxOutput, double integralZone, double maxIntegralAccum, double kf, FeedForwardMode feedforwardmode, FeedForwardBehavior feedforwardbehavior, double unitsPerQuarterWave) {
  // TerriblePID turnPID = new TerriblePID(kp,
  //   RobotConfig.auto.turnInPlace.max_turn_speed
  // );

  TerriblePID turnPID = new TerriblePID(kp, ki, kd, 0, -1, 1, integralZone, maxIntegralAccum, 0, null, null);


  /**
   * Turn a specified number of degrees in the default auto gear.
   * This constructor will default to taking the angle relative to
   * the robot's angle when the command is initialized, not the 
   * absolute angle. If you want to specify, use a bool as the second
   * argument to specify if the angle should be interpreted as absolute 
   * or not.
   * @param target_angle
   */
  public TurnInPlace(double target_angle) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    this.target_angle = target_angle;
  }

  /**
   * Turn a specified number of degrees in the default auto gear. 
   * The angle passed is an absolute angle relative to the 
   * angle upon autonomous init.
   * @param target_angle
   * @param isAbsolute
   */
  public TurnInPlace(double target_angle, boolean isAbsolute) {
    this.isAbsolute = isAbsolute;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    this.target_angle = target_angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    starting_angle = Robot.drivetrain.getGyro() ;

    // If the angle is relative (which it should not be), setup target angle.
    // Otherwise the angle is absolute (relative to auto init) so we don't care.
    if (!(isAbsolute)){ // if isAbsolute is false, and we want a relative angle
      target_angle = target_angle + starting_angle;
    }

    turnPID.setSetpoint(target_angle);
    System.out.println("Turn in place init'ed!");



  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    output = turnPID.update(Robot.drivetrain.getGyro());
    raw_left = EncoderLib.distanceToRaw(output, RobotConfig.driveTrain.left_wheel_effective_diameter, RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);
    raw_right = (-1) * EncoderLib.distanceToRaw(output, RobotConfig.driveTrain.right_wheel_effective_diameter, RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);
    Robot.drivetrain.setPowers(output, -output);
    System.out.println(String.format("Turn in place execute! Target: %s Gyro output: %s,Output: %s, Raw left: %s Raw right %s", turnPID.getSetpoint(), Robot.drivetrain.getGyro(), output, raw_left, raw_right));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if ( (Math.abs(Robot.gyro.getRate() ) < RobotConfig.turn_auto_angular_velocity_tolerence)
    //   && (Math.abs(Robot.drivetrain.getGyro()) < RobotConfig.turn_auto_angle_tolerence)) {
    //     return true;
    //   } else { return false; }

    // TODO so this is how a return works
    // return ( (Math.abs(Robot.gyro.getRate() ) < RobotConfig.auto.tolerences.angular_velocity_tolerence)
    //   && (Math.abs(Robot.drivetrain.getGyro() - target_angle) < RobotConfig.auto.tolerences.angle_tolerence));
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
