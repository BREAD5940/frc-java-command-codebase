package frc.robot.commands.subsystems.superstructure.wrist;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;
import frc.robot.states.IntakeAngle;
import frc.robot.subsystems.superstructure.Wrist;
import frc.robot.subsystems.superstructure.Superstructure.iPosition;

/**
* 
*/
public class SetWrist extends Command {
  boolean isInstant;
  double target_angle;
  double position_tolerence = EncoderLib.degreesToRaw(RobotConfig.wrist.wrist_position_tolerence,
  RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);
  double velocity_tolerence = EncoderLib.degreesToRaw(RobotConfig.wrist.wrist_velocity_tolerence,
  RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION); // TODO verify this behavior, maybe omit for now?
  
  /**
  * This constructor takes a target angle in degrees and the boolean isInstant.
  * If you want thsi command to return instantly, use the one argument
  * constructor unless you want to be verbose. If isInstant is true, this command
  * will only return if both position and angular velocity are within thresholds.
  * 
  * @param target_angle in degrees from zero
  * @param isInstant    boolean
  */
  public SetWrist(double target_angle, boolean isInstant) {
    this.isInstant = isInstant;
    this.target_angle = target_angle;
    // requires(Robot.wrist); // reserve the wrist subsystem
  }
  
  /**
  * Boring version of the set wrist constructor. Defaults to instant return
  * without waiting for angle. If you want to wait for the wrist to reach the
  * target angle, set the second argument to false.
  * 
  * @param target_angle in degrees from zero
  */
  public SetWrist(double target_angle) {
    this.isInstant = true;
    // TODO review if the default behavior of the wrist (to not wait for angle
    // before returning) is desirable
    this.target_angle = target_angle;
    // requires(Robot.wrist); // reserve the wrist subsystem
  }

  public SetWrist(IntakeAngle pos){
    //TODO maybe actually make this do something?
  }

  public SetWrist(double targetAngle1, double targetAngle2){
    //TODO make this do a thing
  }

  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.wrist.setAngle(target_angle);
  }
  
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println("Current wrist angle: " + Robot.wrist.getAngle() + "
    // Target wrist angle: " + target_angle);
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if (isInstant || //if command has to run instantly, return true - otherwise
    // check all the conditions
    // (( Math.abs(Robot.wrist.getAngle() - target_angle ) < position_tolerence )
    // && (Math.abs(Robot.wrist.getAngularVelocity()) < velocity_tolerence))) {
      // return true;
      // }
      // else {
        // return false;
        // }
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
    