package frc.robot;

import frc.math.Pose2d;
import frc.math.Translation2d;
import frc.math.Util;
import frc.robot.lib.Units;

public class Constants {
  /* Graciously borrowed from 254 2018 */

  // Trajectory stuff
  public static final double kPathingAngleTolerence = 4;
  public static final Translation2d kPathingPositionTolerence = new Translation2d(Units.inches_to_meters(6), Units.inches_to_meters(3));

  // Wheels
  public static final double kDriveWheelTrackWidthInches = 25.54;
  public static final double kDriveWheelDiameterInches = 3.92820959548 * 0.99;
  public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
  public static final double kTrackScrubFactor = 1.0;  // Tune me!

  // Tuned dynamics
  public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
  public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
  public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
  public static final double kDriveVIntercept = 1.055;  // V
  public static final double kDriveKv = 0.135;  // V per rad/s
  public static final double kDriveKa = 0.012;  // V per rad/s^2

}