package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Time;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;

import frc.robot.lib.motion.Util;

public class Constants {
  /* Graciously borrowed from 5190*/
  public static final double kRobotMass = 50 /* Robot, kg */ + 5f /* Battery, kg */ + 2f /* Bumpers, kg */;
  public static final double kRobotMomentOfInertia = 10.0; // kg m^2 // TODO Tune
  public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)

  public static final double kWheelRadius = Util.toMeters(2f/12f);// meters. TODO tune
  public static final double kTrackWidth = Util.toMeters(26f/12f);// meters
  
  public static final double kStaticFrictionVoltage = 1.8; // Volts TODO tune
  public static final double kVDrive = 0.16; // Volts per radians per second - Calculated with https://docs.google.com/spreadsheets/d/1I2WgbKy0QJsedhJbi41485n7QCGcKez666ui2kPl23I/edit#gid=0 for 15ft/sec (high gear)
  public static final double kADrive = 0.0716; // Volts per radians per second per second TODO tune

  /* Ramsete constants */
  public static final double kDriveBeta = 1.5; // Inverse meters squared
  public static final double kDriveZeta = 0.9; // Unitless dampening co-efficient

  /* Pure Pursuit constants */
  public static final double kLat = 0.05f;
  public static final Time kLookaheadTime = TimeUnitsKt.getSecond(0.1);
  public static final Length kMinLookaheadDistance = LengthKt.getFeet(2);

}