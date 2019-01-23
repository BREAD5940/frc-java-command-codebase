package frc.robot.commands.subsystems.drivetrain;

import frc.robot.lib.motion.planners.DriveMotionPlanner.FollowerType;
import frc.robot.lib.obj.DriveSignal;
import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.motion.Odometer;
import frc.robot.lib.motion.Output;
import frc.robot.lib.motion.PathfinderTrajectory;
import frc.robot.lib.motion.planners.DriveMotionPlanner;

public class RunDriveMotionPlanner extends Command {

  private DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner();

  FollowerType followerType = FollowerType.NONLINEAR_FEEDBACK;

  private boolean mOverrideTrajectory = false;

  public RunDriveMotionPlanner(PathfinderTrajectory trajectory) {
    requires(Robot.drivetrain);
    mMotionPlanner.setTrajectory(trajectory);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    final double now = Timer.getFPGATimestamp();

    Output output = mMotionPlanner.update(now, Odometer.getInstance().getPose());

    // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

    // mPeriodicIO.error = mMotionPlanner.error();
    // mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

    if (!mOverrideTrajectory) {
      Robot.drivetrain.setVelocity(
        new DriveSignal(DriveTrain.radiansPerSecondToTicksPer100ms(output.left_velocity), DriveTrain.radiansPerSecondToTicksPer100ms(output.right_velocity)),
                
        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0),
                DriveTrain.radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0,
                DriveTrain.radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0         
      );

        // mPeriodicIO.left_accel = DriveTrain.radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        // mPeriodicIO.right_accel = DriveTrain.radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
    } else {
        Robot.drivetrain.setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE, 0, 0);
        // mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
        // Robot.drivetrain.setMode(NeutralMode.Brake);
        // Robot.drivetrain.setSpeeds(0,0);
    }
// } else {
    // DriverStation.reportError("Drive is not in path following state", false);
// }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
