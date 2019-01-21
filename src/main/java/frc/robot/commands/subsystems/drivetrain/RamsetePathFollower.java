package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.math.Util;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.Logger;
import frc.robot.lib.motion.DriveMotorState;
import frc.robot.lib.motion.Odometer;
import frc.robot.lib.motion.followers.RamseteFollower;
import frc.robot.subsystems.DriveTrain.Gear;

/**
 * Follow a path using a Ramsete Follower. Paths are generated 
 * by pathweaver and uploaded via CSV.
 * 
 * @author Matthew Morley
 */
public class RamsetePathFollower extends Command {

  private RamseteFollower mFollower;
  // private Gear gear;
  private double start;
  private DriveMotorState mDriveMotorState;

  public RamsetePathFollower(String trajectory, boolean setInitialOdometry, Gear gear) {
    requires(Robot.drivetrain);
    mFollower = new RamseteFollower(Util.toMeters(RobotConfig.driveTrain.wheel_base) , trajectory);
    // this.gear = gear;
    Logger.log("Ramsete follower instantiated");
    Robot.drivetrain.setGear(gear);
    start = System.nanoTime();
    if (setInitialOdometry) {
      Odometer.getInstance().setPose( mFollower.getInitialPose() );
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // we don't need to set odometry because it's called in update
    Logger.log("getting next drive signal");
    mDriveMotorState = mFollower.getNextDriveSignal();
    Logger.log("drive signal: ", mDriveMotorState);
    Robot.drivetrain.setFeetPerSecond( Util.toMeters( mDriveMotorState.leftVel), Util.toMeters(mDriveMotorState.rightVel));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.log("Follower is done!");
    System.out.println("Time to completion: " + ((System.nanoTime()-start)/1000000));
    Robot.drivetrain.setSpeeds(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
