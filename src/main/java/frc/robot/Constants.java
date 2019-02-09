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
	public static final double kVDriveLeftLow = 0.265 * 1d; // Volts per radians per second - Calculated emperically
	public static final double kADriveLeftLow = 0.027 * 1d; // Volts per radians per second per second TODO tune
	public static final double kVInterceptLeftLow = 3.5 * 0.4d; // Volts - tuned!

	public static final double kVDriveRightLow = 0.275 * 1d; // Volts per radians per second - Calculated emperically
	public static final double kADriveRightLow = 0.0286 * 1d; // Volts per radians per second per second TODO tune
	public static final double kVInterceptRightLow = 3.5 * 0.4d; // Volts - tuned!

	public static final DCMotorTransmission kLeftTransmissionModelLowGear = new DCMotorTransmission(1 / kVDriveLeftLow,
			kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeftLow),
			kVInterceptLeftLow);

	public static final DCMotorTransmission kRightTransmissionModelLowGear = new DCMotorTransmission(1 / kVDriveRightLow,
			kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRightLow),
			kVInterceptRightLow);

	public static final double kVDriveLeftHigh = 0.139 * 1d; // Volts per radians per second - Calculated emperically
	public static final double kADriveLeftHigh = 0.035 * 1d; // Volts per radians per second per second 
	public static final double kVInterceptLeftHigh = 4 * 0.4d; // Volts - tuned!

	public static final double kVDriveRightHigh = 0.1375 * 1d; // Volts per radians per second - Calculated emperically
	public static final double kADriveRightHigh = 0.03675 * 1d; // Volts per radians per second per second 
	public static final double kVInterceptRightHigh = 4 * 0.4d; // Volts - tuned!

	public static final DCMotorTransmission kLeftTransmissionModelHighGear = new DCMotorTransmission(1 / kVDriveLeftHigh,
			kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeftHigh),
			kVInterceptLeftHigh);

	public static final DCMotorTransmission kRightTransmissionModelHighGear = new DCMotorTransmission(1 / kVDriveRightHigh,
			kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRightHigh),
			kVInterceptRightHigh);


	/* Ramsete constants */
	public static final double kDriveBeta = 1.5 * 1d; // Inverse meters squared
	public static final double kDriveZeta = 0.9 * 1d; // Unitless dampening co-efficient

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
