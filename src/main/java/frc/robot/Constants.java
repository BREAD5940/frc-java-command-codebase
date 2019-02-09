package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Time;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;

import com.team254.lib.physics.DCMotorTransmission;

import frc.robot.lib.motion.Util;

public class Constants {
	/* Graciously borrowed from 5190*/
	public static final double kRobotMass = 50 /* Robot, kg */ + 5f /* Battery, kg */ + 2f /* Bumpers, kg */;
	public static final double kRobotMomentOfInertia = 10.0; // kg m^2 // TODO Tune
	public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)

	public static final double kWheelRadius = Util.toMeters(2f / 12f);// meters. TODO tune
	public static final double kTrackWidth = Util.toMeters(26f / 12f);// meters

	// high gear: 7.5 rad/s
	// low gear: 4.3 rad/s
	public static final double kVDriveLeft = 0.2518 * 1d; // Volts per radians per second - Calculated emperically
	public static final double kADriveLeft = 0.03 * 1d; // Volts per radians per second per second TODO tune
	public static final double kVInterceptLeft = 0.8537; // Volts - tuned!

	public static final double kVDriveRight = 0.2631 * 1d; // Volts per radians per second - Calculated emperically
	public static final double kADriveRight = 0.0286 * 1d; // Volts per radians per second per second TODO tune
	public static final double kVInterceptRight = 0.893; // Volts - tuned!

	public static final DCMotorTransmission kLeftTransmissionModel = new DCMotorTransmission(1 / kVDriveLeft,
			kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeft),
			kVInterceptLeft);

	public static final DCMotorTransmission kRightTransmissionModel = new DCMotorTransmission(1 / kVDriveRight,
			kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRight),
			kVInterceptRight);

	/* Ramsete constants */
	public static final double kDriveBeta = 1.5; // Inverse meters squared
	public static final double kDriveZeta = 0.9; // Unitless dampening co-efficient

	/* Pure Pursuit constants */
	public static final double kLat = 0.05f;
	public static final Time kLookaheadTime = TimeUnitsKt.getSecond(0.1);
	public static final Length kMinLookaheadDistance = LengthKt.getFeet(2);

	/* Wrist stuff */
	public static final Length kWristLength = LengthKt.getInch(6); //distance from joint to COM
	public static final Mass kWristMass = MassKt.getLb(15);
	public static final double kWristSpeedPerVolt = 0.21d; // radians/sec/volt
	public static final double kWristTorquePerVolt = 47.33; // Newton meters per volt, stall
	public static final double kWristStaticFrictionVoltage = 0; // volts, TODO tune

	/* Elbow stuff */
	public static final Length kElbowLength = LengthKt.getInch(8); //distance from joint to COM
	public static final Mass kElbowMass = MassKt.getLb(3);
	public static final double kElbowSpeedPerVolt = 0.17d; // radians/sec/volt
	public static final double kElbowTorquePerVolt = 55; // Newton meters per volt, stall
	public static final double kElbowStaticFrictionVoltage = 0; // volts, TODO tune

}
