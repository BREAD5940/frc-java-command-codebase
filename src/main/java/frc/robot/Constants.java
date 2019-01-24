package frc.robot;

import frc.math.Pose2d;
import frc.math.Translation2d;
import frc.math.Util;
import frc.robot.lib.Units;

public class Constants {
  /* Graciously borrowed from 5190*/
  public static final double kRobotMass = 54.53 /* Robot */ + 5.669 /* Battery */ + 7; /* Bumpers */ // kg
  public static final double kRobotMomentOfInertia = 10.0; // kg m^2 // TODO Tune
  public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)

  public static final double kStaticFrictionVoltage = 1.8; // Volts
  public static final double kVDrive = 0.115; // Volts per radians per second
  public static final double kADrive = 0.0716; // Volts per radians per second per second

  public static final double kDriveBeta = 1.4; // Inverse meters squared
  public static final double kDriveZeta = 0.9; // Unitless dampening co-efficient

  public static final double kWheelRadius = 2.92;// inches
  public static final double kTrackWidth = 0.61;// meters

  public static final double kDriveLowGearVelocityKa = 0.1; // TODO tune!

  // public static final double kDriveBeta = 1.4; // Inverse meters squared
  // public static final double kDriveZeta = 0.9; // Unitless dampening co-efficient

}