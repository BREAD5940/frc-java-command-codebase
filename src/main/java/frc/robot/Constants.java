package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Time;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;

import frc.math.Pose2d;
import frc.math.Translation2d;
import frc.math.Util;
import frc.robot.lib.Units;

public class Constants {
  /* Graciously borrowed from 5190*/
  public static final double kRobotMass = 25.8f /* Robot, kg */ + 5f /* Battery, kg */ + 0f; /* Bumpers */ // kg
  public static final double kRobotMomentOfInertia = 10.0; // kg m^2 // TODO Tune
  public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)

  public static final double kWheelRadius = Util.toMeters(2f/12f);// inches. TODO tune
  public static final double kTrackWidth = Util.toMeters(24f/12f);// meters
  
  public static final double kStaticFrictionVoltage = 1.8; // Volts
  public static final double kVDrive = 0.115; // Volts per radians per second
  public static final double kADrive = 0.0716; // Volts per radians per second per second

  public static final double kDriveBeta = 1.4; // Inverse meters squared
  public static final double kDriveZeta = 0.9; // Unitless dampening co-efficient

  public static final double kLat = 0f;
  public static final Time kLookaheadTime = TimeUnitsKt.getSecond(0.1);
  public static final Length kMinLookaheadDistance = LengthKt.getFeet(1);


  public static final double kDriveLowGearVelocityKa = 0.1; // TODO tune!

  // public static final double kDriveBeta = 1.4; // Inverse meters squared
  // public static final double kDriveZeta = 0.9; // Unitless dampening co-efficient

}