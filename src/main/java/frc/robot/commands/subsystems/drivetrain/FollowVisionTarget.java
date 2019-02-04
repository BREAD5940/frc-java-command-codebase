package frc.robot.commands.subsystems.drivetrain;

import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;
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

  private TerriblePID forwardPID = new TerriblePID(
    RobotConfig.auto.followVisionTarget.forward.kp,
    RobotConfig.auto.drive_auto_forward_velocity_min,
    RobotConfig.auto.drive_auto_forward_velocity_max 
  );
  private TerriblePID turnPID = new TerriblePID(
    RobotConfig.auto.followVisionTarget.turn.kp,
    RobotConfig.auto.followVisionTarget.turn.min_turn_speed,
    RobotConfig.auto.followVisionTarget.turn.max_turn_speed
  );
  
  /**
   * Follow a limelight vision target. Move toward the target at the set speed 
   * and timeout after a set time. Tracks only angle, not range!
   * @param speed between -1 and 1
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
   * @param speed to track forwards at between -1 and 1
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
      // if (followRange) {
      //   forwardSpeed = forwardPID.update(Robot.limelight.getTargetArea());
      // } else {
      //   forwardSpeed = forwardPID.update(Robot.drivetrain.getLeftVelocity());
      // }
      // turnSpeed = turnPID.update(Robot.limelight.getData()[1]);
      double[] data = Robot.limelight.getData();
      double limelightData = data[1];
      double sizeData = data[3];
      turnSpeed = limelightData * (1/30) ;

      forwardSpeed = 0;
      
      leftSpeedRaw = EncoderLib.distanceToRaw(forwardSpeed + turnSpeed, RobotConfig.driveTrain.left_wheel_effective_diameter / 12, 
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) / 10;
      rightSpeedRaw = EncoderLib.distanceToRaw(forwardSpeed - turnSpeed, RobotConfig.driveTrain.right_wheel_effective_diameter / 12, 
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) / 10;

      // System.out.println("FORWARD PID: Setpoint: " + forwardPID.getSetpoint() + " Measured: " + Robot.drivetrain.getLeft().getFeet() + 
        // " Error: " + forwardPID.getError() + " OUTPUT VELOCITY (ft/s): " + forwardPID.getOutput());
      // System.out.println("TURN PID: Setpoint: " + turnPID.getSetpoint()+ 
        // " Error: " + turnPID.getError() + " OUTPUT VELOCITY (ft/s): " + turnPID.getOutput());

      System.out.println("Limelight data: " + limelightData + " Turn speed: " + turnSpeed);

      // double targetSizeSetpoint = 2;
      double distanceRatio = targetPercentOfFrame - sizeData;

      double forwardSpeed = distanceRatio * 0.5;

      if ( forwardSpeed > 0.5 ) { forwardSpeed = 0.5;}
      if ( forwardSpeed < -0.5 ) { forwardSpeed = -0.5;}

 
      // Robot.drivetrain.setSpeeds(leftSpeedRaw, rightSpeedRaw);
      Robot.drivetrain.setPowers(forwardSpeed + limelightData / 20, forwardSpeed - limelightData / 20);

    } else {
      System.out.println("No targets currently being tracked! Can't track thin air, can I?");
      Robot.drivetrain.setPowers(0, 0);
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
