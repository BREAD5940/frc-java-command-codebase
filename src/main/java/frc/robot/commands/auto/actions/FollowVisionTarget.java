package frc.robot.commands.auto.actions;

import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;
import frc.robot.lib.Logger;
import frc.robot.lib.TerriblePID;
import edu.wpi.first.wpilibj.command.Command;


/**
 * Follow a vision target tracked by a limelight
 */
public class FollowVisionTarget extends Command {
  double timeout,
  targetSpeed,
  targetPercentOfFrame,
  angleDeltaX,
  angleDeltaY,
  forwardSpeed,
  turnSpeed,
  leftSpeedRaw,
  rightSpeedRaw;
  boolean followRange = false;

  boolean hadTarget = false;
  double lastKnownYaw;

  private TerriblePID forwardPID = new TerriblePID(
    RobotConfig.auto.followVisionTarget.forward.kp,
    RobotConfig.auto.drive_auto_forward_velocity_max 
  );
  private TerriblePID turnPID = new TerriblePID(
    RobotConfig.auto.followVisionTarget.turn.kp,
    RobotConfig.auto.followVisionTarget.turn.max_turn_speed
  );
  
  /**
   * Follow a limelight vision target. Move toward the target at the set speed 
   * and timeout after a set time. Tracks only angle, not range!
   * @param speed
   * @param timeout
   */
  public FollowVisionTarget(double speed, double timeout) {
    this.timeout = timeout;
    this.targetSpeed = speed;
    requires(Robot.drivetrain);
  }

  /**
   * Follow a limelight vision target. Track both range and angle
   * of the target to the limelight
   * @param speed to track forwards at
   * @param targetPercentOfFrame percent of frame taken up by target
   * @param timeout
   */
  public FollowVisionTarget(double speed, double targetPercentOfFrame, double timeout) {
    this.timeout = timeout;
    this.targetSpeed = speed;
    this.targetPercentOfFrame = targetPercentOfFrame;
    this.followRange = true;
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(timeout);
    turnPID.setSetpoint(0);
    forwardPID.setMaxOutput(targetSpeed);

    if (followRange){ 
      forwardPID.setSetpoint(targetPercentOfFrame);
      forwardPID.setKpGain(RobotConfig.auto.followVisionTarget.forward.kp_rangeMode);
    } 
    else { forwardPID.setSetpoint(targetSpeed);  }

    angleDeltaX = Robot.limelight.getDx();
    targetPercentOfFrame = Robot.limelight.getTargetArea();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if ( Robot.limelight.getData()[0] != 0 ) {
      hadTarget = true;

      double[] data = Robot.limelight.getData();
      double limelightData = data[1];
      double sizeData = data[3];

      lastKnownYaw = limelightData;
      turnSpeed = limelightData * (1/45) ;

      forwardSpeed = 0;
      
      leftSpeedRaw = EncoderLib.distanceToRaw(forwardSpeed + turnSpeed, RobotConfig.driveTrain.left_wheel_effective_diameter / 12, 
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) / 10;
      rightSpeedRaw = EncoderLib.distanceToRaw(forwardSpeed - turnSpeed, RobotConfig.driveTrain.right_wheel_effective_diameter / 12, 
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) / 10;


      // double targetSizeSetpoint = 0.8;
      double distanceRatio = 4.8 - sizeData;

      double forwardSpeed = distanceRatio * 1;

      if ( forwardSpeed > 0.5 ) { forwardSpeed = 0.5;}
      if ( forwardSpeed < -0.5 ) { forwardSpeed = -0.5;}

      System.out.println("forward speed: " + forwardSpeed + " Turn speed: " + turnSpeed);

      // Robot.drivetrain.setSpeeds(leftSpeedRaw, rightSpeedRaw);
      Robot.drivetrain.setPowers(forwardSpeed + limelightData / 20, forwardSpeed - limelightData / 20);

    } else {
      if (hadTarget) {
        // just spin in a circle in the last knwon direction
        Logger.log("I can't see anything and I *had* a target, lets just spin in a circle like a chump");
        double speed = (lastKnownYaw > 0) ? 0.3 : -0.3;
        Robot.drivetrain.setPowers(speed, -speed);
      }
    }
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
