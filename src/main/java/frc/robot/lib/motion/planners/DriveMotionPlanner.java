package frc.robot.lib.motion.planners;

import frc.math.Pose2d;
import frc.math.Util;
import frc.robot.Constants;
import frc.robot.lib.Units;
import frc.robot.lib.motion.DCMotorTransmission;
import frc.robot.lib.motion.DifferentialDrive;
import frc.robot.lib.motion.Output;
import frc.robot.lib.motion.PathfinderTrajectory;
import frc.robot.lib.motion.TimedTrajectorySegment;
import frc.robot.lib.motion.followers.PathfinderTrajectoryIterator;
import jaci.pathfinder.Trajectory;

public class DriveMotionPlanner {
  private static final double kMaxDx = 2.0;
  private static final double kMaxDy = 0.25;
  private static final double kMaxDTheta = Math.toRadians(5.0);

  public enum FollowerType {
      FEEDFORWARD_ONLY,
      PURE_PURSUIT,
      PID,
      NONLINEAR_FEEDBACK
  }

  PathfinderTrajectoryIterator mCurrentTrajectory;

  FollowerType mFollowerType = FollowerType.NONLINEAR_FEEDBACK;

  final DifferentialDrive mModel;

  final DCMotorTransmission transmission;

  Pose2d mError = Pose2d.identity();

  double mLastTime = Double.POSITIVE_INFINITY;

  Output mOutput = new Output();

  double mDt;

  DifferentialDrive.ChassisState prev_velocity_ = new DifferentialDrive.ChassisState();

  // PathfinderTrajectoryIterator mCurrentTrajectory;

  public DriveMotionPlanner() {
    transmission = new DCMotorTransmission(
      1.0 / Constants.kDriveKv,
      Units.inches_to_meters(Constants.kDriveWheelRadiusInches) * Units.inches_to_meters(Constants
              .kDriveWheelRadiusInches) * Constants.kRobotLinearInertia / (2.0 * Constants.kDriveKa),
      Constants.kDriveVIntercept);
    mModel = new DifferentialDrive(
      Constants.kRobotLinearInertia,
      Constants.kRobotAngularInertia,
      Constants.kRobotAngularDrag,
      Units.inches_to_meters(Constants.kDriveWheelDiameterInches / 2.0),
      Units.inches_to_meters(Constants.kDriveWheelTrackWidthInches / 2.0 * Constants.kTrackScrubFactor),
      transmission, transmission
    );
  }

  public void setTrajectory( PathfinderTrajectory _trajectory_ ) {
    mCurrentTrajectory = new PathfinderTrajectoryIterator(_trajectory_);
  }

  public Output update(double timestamp, Pose2d current_state) {
    if (mCurrentTrajectory == null) return new Output();

    if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
      mLastTime = timestamp;
    }

    mDt = timestamp - mLastTime;
    mLastTime = timestamp;

    TimedTrajectorySegment mSetpoint = mCurrentTrajectory.advance(mDt);

    if (!mCurrentTrajectory.isDone(current_state)) {
      
      // Generate feedforward voltages.
      System.out.println("mSetpoint.velocity is: " + mSetpoint.velocity);
      double velocity_m = mSetpoint.velocity;
      double curvature_m = Units.meters_to_inches(mSetpoint.pose().getCurvature());
      double dcurvature_ds_m = Units.meters_to_inches(Units.meters_to_inches(mSetpoint.pose()
              .getDCurvatureDs()));
      double acceleration_m = /*Units.inches_to_meters*/(mSetpoint.acceleration);
      DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
              new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m),
              new DifferentialDrive.ChassisState(acceleration_m,
                      acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
      mError = current_state.inverse().transformBy(mSetpoint.pose().getPose());

      if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
          mOutput = new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics
                  .wheel_acceleration.left, dynamics.wheel_acceleration.right, dynamics.voltage
                  .left, dynamics.voltage.right);
      // } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
      //     mOutput = updatePurePursuit(dynamics, current_state);
      // } else if (mFollowerType == FollowerType.PID) {
      //     mOutput = updatePID(dynamics, current_state);
      } else if (mFollowerType == FollowerType.NONLINEAR_FEEDBACK) {
          mOutput = updateNonlinearFeedback(dynamics, current_state);
      }
    } else {
      mOutput = new Output();
    }
    return mOutput;
  }

  protected Output updateNonlinearFeedback(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {

    // Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
    final double kBeta = 2.0;  // >0.
    final double kZeta = 0.7;  // Damping coefficient, [0, 1].

    // Compute gain parameter.
    final double k = 2.0 * kZeta * Math.sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity
            .linear + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular);

    // Compute error components.
    final double angle_error_rads = mError.getRotation().getRadians();
    final double sin_x_over_x = Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ?
            1.0 : mError.getRotation().sin() / angle_error_rads;
    final DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState(
            dynamics.chassis_velocity.linear * mError.getRotation().cos() +
                    k * Units.inches_to_meters(mError.getTranslation().x()),
            dynamics.chassis_velocity.angular + k * angle_error_rads +
                    dynamics.chassis_velocity.linear * kBeta * sin_x_over_x * Units.inches_to_meters(mError
                            .getTranslation().y()));

    // Compute adjusted left and right wheel velocities.
    dynamics.chassis_velocity = adjusted_velocity;
    dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);

    dynamics.chassis_acceleration.linear = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.linear - prev_velocity_
            .linear) / mDt;
    dynamics.chassis_acceleration.angular = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.angular - prev_velocity_
            .angular) / mDt;

    prev_velocity_ = dynamics.chassis_velocity;

    DifferentialDrive.WheelState feedforward_voltages = mModel.solveInverseDynamics(dynamics.chassis_velocity,
            dynamics.chassis_acceleration).voltage;

    return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration
            .left, dynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);

  }
}