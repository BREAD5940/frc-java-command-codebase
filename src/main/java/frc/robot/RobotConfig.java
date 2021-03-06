package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;

import frc.robot.subsystems.DriveTrain.Gear;

public class RobotConfig {

	/**
	* Joystick configuration
	*/

	public class controls {
		public static final int primary_joystick_port = 0;
		public static final int control_pannel_port = 2; // TODO actually confirm this
		public static final int forward_axis = xboxmap.Axis.LEFT_JOYSTICK_Y;
		public static final int turn_axis = xboxmap.Axis.RIGHT_JOYSTICK_X;
		public static final boolean driving_squared = false;
		public static final int shift_down_button = xboxmap.Buttons.LB_BUTTON;
		public static final int shift_up_button = xboxmap.Buttons.RB_BUTTON;
		public static final int intakeAxis = xboxmap.Axis.RIGHT_TRIGGER;
		public static final int outtakeAxis = xboxmap.Axis.LEFT_TRIGGER;
		public static final int intakeOpen = xboxmap.Buttons.LB_BUTTON;
		public static final int intakeClose = xboxmap.Buttons.RB_BUTTON;
		public static final int secondary_joystick_port = 1;
		public static final int xbox_elevator_axis = 1; // TODO fix elevator axis for xbox
		public static final int throttle_elevator_axis = 2; // TODO set this axis correctly!
		public static final double throttle_minimum_value = 0; // TODO fix elevator minimum height!
		public static final double throttle_maximum_value = 1; // TODO fix elevator maximum height
	}

	public class wrist {
		/**
		* Wrist configuration
		*/
		public static final int m_wrist_talon_port = 40; // TODO fix wrist talon port!!
		public static final int s_wrist_talon_port = 41; // TODO fix wrist talon port!!
		public static final float minimum_wrist_angle = 0;
		public static final float maximum_wrist_angle = 90;
		public static final double wrist_position_tolerence = 5; // 5 degree tolerence, be sure to convert to raw!
		// public static final double wrist_velocity_tolerence = 2; // 2 degrees per second??

		public static final boolean talon_direction_inverted = false;
		public static final boolean talon_encoder_inverted = false;

		// public static final double kStaticCoefficient = 0.3;
		public class talonConfig {
			public static final double position_kp = 0.1;
			public static final double position_ki = 0;
			public static final double position_kd = 0;
			public static final double gravity_ff = 0.1; // the gain on the gravity feedforward
			public static final double software_position_kp = 0.1;
			public static final double software_position_ki = 0;
			public static final double software_position_kd = 0;
			public static final double software_position_kf = 0;
			public static final int position_izone = 200;
			public static final double position_max_integral = 0.5;
		}
	}

	public static class driveTrain {
		/**
		* Robot configuration
		*/

		/** Drivetrain width in feet */
		public static final double wheel_base = 2.5;
		public static final double left_wheel_effective_diameter = 6; // units are in inches, TODO tune this!
		public static final double right_wheel_effective_diameter = 6; // units are in inches, TODO tune this!

		public static final Length left_radius = LengthKt.getInch(2);
		public static final Length right_radius = LengthKt.getInch(2);

		// Set speeds for teleop
		public static final double max_forward_speed_high = 7; // Feet per second forward velocity
		public static final double max_turn_speed_high = 7; // Max turn speed in degrees per second
		public static final double max_forward_speed_low = 3; // Feet per second forward velocity
		public static final double max_turn_speed_low = 3; // Max turn speed in degrees per second
		public static final double tipThreshold = 5; //max ft/sec the robot can go with the elevator at full speed w/o tipping FIXME find the actual num

		// Encoder stuff, dunno where else to put this
		// public static final double VELOCITY_PULSES_PER_ROTATION = 409.6f;
		public static final double POSITION_PULSES_PER_ROTATION = 4096;

		public static final NativeUnit kDriveSensorUnitsPerRotation = NativeUnitKt.getSTU(4096);

		public static final NativeUnitLengthModel LEFT_NATIVE_UNIT_LENGTH_MODEL = new NativeUnitLengthModel(kDriveSensorUnitsPerRotation, left_radius);
		public static final NativeUnitLengthModel RIGHT_NATIVE_UNIT_LENGTH_MODEL = new NativeUnitLengthModel(kDriveSensorUnitsPerRotation, right_radius);

		// Pathfinder shit
		public static final double left_static_kv = 0.05; //TODO TUNE THIS! the voltage required to get the robot moving/overcome static friction
		public static final double right_static_kv = 0.05; //TODO TUNE THIS! the voltage required to get the robot moving/overcome static friction

		public class leftTalons {
			/**
			* Left side of drivetrain PID constants and setup
			*/
			public static final int m_left_talon_port = 1;
			public static final int s_left_talon_port = 2;
			public static final boolean m_left_inverted = false;
			// sets kp, ki, kd and kf terms for master left in velocity mode 
			public static final double velocity_kp_low = 0.45;
			public static final double velocity_ki_low = 0.0;
			public static final double velocity_kd_low = velocity_kp_low * 20f;
			public static final double velocity_kf_low = 0.0;
			public static final int velocity_izone_low = 800;
			public static final double velocity_max_integral_low = 500000;
			public static final double position_kp_low = 0.0;
			public static final double position_ki_low = 0;
			public static final double position_kd_low = 0;
			public static final double position_kf_low = 0.0d;
			public static final int position_izone_low = 800;
			// public static final double position_max_integral_low = 1;
			public static final double velocity_kp_high = 1.2f;
			public static final double velocity_ki_high = 0;
			public static final double velocity_kd_high = 10f;
			public static final double velocity_kf_high = 0.0;
			public static final int velocity_izone_high = 800;
			public static final double velocity_max_integral_high = 300;
			public static final double position_kp_high = 2;
			public static final double position_ki_high = 0;
			public static final double position_kd_high = 20;
			public static final double position_kf_high = 0.0;
			public static final int position_izone_high = 300;
			// public static final double position_max_integral_high = 5;

			// kv/ka settings for pathfinder
			public static final double velocity_kv_high = 0.63768115942028985507246376811594;
			public static final double velocity_ka_high = 1.0 / 30; // borrowed from robolancers, so idk if this is a good value
			public static final double velocity_kv_low = 0.63768115942028985507246376811594;
			public static final double velocity_ka_low = 1.0 / 30; // borrowed from robolancers, so idk if this is a good number
		}

		public class rightTalons {
			/**
			* Right side of drivetrain PID constants and setup
			*/
			public static final int m_right_talon_port = 3;
			public static final int s_right_talon_port = 4;
			public static final double velocity_kp_low = leftTalons.velocity_kp_low;
			public static final double velocity_ki_low = leftTalons.velocity_ki_low;
			public static final double velocity_kd_low = leftTalons.velocity_kd_low;
			public static final double velocity_kf_low = leftTalons.velocity_kf_low;
			public static final int velocity_izone_low = leftTalons.velocity_izone_low;
			public static final double velocity_max_integral_low = 5;
			public static final double position__kp_low = leftTalons.position_kp_low;
			public static final double position__ki_low = leftTalons.position_ki_low;
			public static final double position__kd_low = leftTalons.position_kd_low;
			public static final double position__kf_low = leftTalons.position_kf_low;
			// public static final int position_izone_low = 5;
			public static final double position__max_integral_low = 1;
			public static final double velocity_kp_high = leftTalons.velocity_kp_high;
			public static final double velocity_ki_high = leftTalons.velocity_ki_high;
			public static final double velocity_kd_high = leftTalons.velocity_kd_high;
			public static final double velocity_kf_high = leftTalons.velocity_kf_high;
			public static final int velocity_izone_high = leftTalons.velocity_izone_high;
			public static final double velocity_max_integral_high = 5;
			public static final double position__kp_high = leftTalons.position_kp_high;
			public static final double position__ki_high = leftTalons.position_ki_high;
			public static final double position__kd_high = leftTalons.position_kd_high;
			public static final double position__kf_high = leftTalons.position_kf_high;
			// public static final int position__izone_high = 5;
			public static final double position__max_integral_high = 5;

			// kv/ka settings for pathfinder
			public static final double velocity_kv_high = 0;
			public static final double velocity_ka_high = 0;
			public static final double velocity_kv_low = 0;
			public static final double velocity_ka_low = 0;
		}
	}

	public static class auto {

		// CHANGE ME TO CHANGE HOW FAST AUTO GOES!!!
		public static final double default_speed = 4;
		public static final double drive_auto_forward_velocity_max = 4; // feet per second target for driving auto
		public static final double drive_auto_forward_velocity_min = -2; // minimum speed for auto drive in ft per sec
		public static Gear auto_gear = Gear.HIGH;

		public static final Length robotRadius = LengthKt.getInch(30); //FIXME this should be the distance from the center of the robot (or the gyro?) to the point on the robot perim furthest from it

		public static class fieldPositions {
			// Positions of objects on the field (ports, etc.) in inches. distances are to the center of the object
			// unless otherwise indicated
			// FIXME change these values to be the distance the elevator has to go to have the intake be at the center of the object

			public static final Length cargoLowGoal = LengthKt.getInch(9);

			public static final Length cargoMiddleGoal = LengthKt.getInch(34);
			public static final Length cargoHighGoal = LengthKt.getInch(54);

			public static final Length hatchLowGoal = LengthKt.getInch(6.25 - 1.25 + 3.5);
			public static final Length hatchMiddleGoal = LengthKt.getInch(31.5);
			public static final Length hatchHighGoal = LengthKt.getInch(53);

			public static final Length shipWall = LengthKt.getInch(31.5); //top of wall
		}

		// String default_auto_gear = "low"; // Set the default gear for auto. If not otherwise specified, this will be used

		public class pathfinder {
			public static final double gyro_correct_kp = 0.2;
		}

		public class tolerences {
			public static final double position_tolerence = 0.5; // units are in incehs
			public static final double velocity_tolerence = 0.5; // units are in inches per second
			public static final double angle_tolerence = 1; // units are in degrees
			public static final double angular_velocity_tolerence = 0.5; // units are in degrees per second
			// public static final double drive_auto_straight_angle_tolerence = 2; // units are in degrees
		}

		public class driveStraight {
			public static final double forward_kp = 2;
			public static final double turn_kp = 0.1;
			public static final double turn_ki = 2;
			public static final double turn_izone = 20; // +- 4 degrees of setpoint for izone
			public static final double turn_integral_max = 5; // Maximum integral weight for turning
			public static final double maximum_turn_speed = 3;
			public static final double minimum_turn_speed = -maximum_turn_speed;
		}

		public class turnInPlace {

			//  public TerriblePID(double kp, double ki, double kd, double kf, double minOutput, double maxOutput, 
			// double integralZone, double maxIntegralAccum, double rampRate, FeedForwardMode forwardMode,
			// FeedForwardBehavior feedforwardbehavior) {

			public static final double kp = 0.1;
			public static final double ki = 0.3;
			public static final double kd = 0.0;
			public static final double kf = 0.0;
			public static final double min_turn_speed = -4; // in ft/sec
			public static final double max_turn_speed = 4; // in ft/sec
			public static final double integral_zone = 10; // 10 degrees izone
			public static final double max_integral = 0.5;
			public static final double ramp_rate = 0.2;
		}

		public class followVisionTarget {
			public class forward {
				public static final double kp = 0.1;
				public static final double kp_rangeMode = 0.1;
			}

			public class turn {
				public static final double kp = 0.05;
				public static final double ki = 0.00;
				public static final double integral_zone = 10; // 10 degrees izone
				public static final double max_integral = 0.3;
				public static final double min_turn_speed = -1; // in ft/sec
				public static final double max_turn_speed = 1; // in ft/sec
			}
		}

	}

	public class limeLight {
		public static final double camera_height = 1; // units are in feet
		public static final double camera_angle = 0; // degrees from horizon - positive is up, negative is down
	}

	public static class ultrasonicSensors {
		public static class sensor1 {
			public static final int echoPort = 0;
			public static final int triggerPort = 0;
		}

		public static class sensor2 {
			public static final int echoPort = 1;
			public static final int triggerPort = 1;
		}

		public static double sensorCenterDistance = 20; // inches
	}

	public class lidar {
		public static final int port = 6;
	}

	/**
	* Elevator configuration
	*/
	public static class elevator {
		public static final double elevator_effective_diameter = 1.27 * 1.6; // TODO fix elevator_effective_diameter!!!! (units must be inches)
		public static final Length elevator_minimum_height = LengthKt.getInch(0f);
		public static final Length elevator_maximum_height = LengthKt.getInch(65f); // changed to inches, TODO verify maximum height

		private static final NativeUnit kSensorUnitsPerRotation = NativeUnitKt.getSTU(4096);
		private static final Length left_radius = LengthKt.getInch(1.5).div(2);

		public static final NativeUnitLengthModel elevatorModel = new NativeUnitLengthModel(kSensorUnitsPerRotation, left_radius);

		public class elevatorTalon {
			/**
			* Elevator configuration
			*/
			public static final int elevator_talon_port = 5;

			public static final double elevator_velocity_kp = 1;
			public static final double elevator_velocity_ki = 0;
			public static final double elevator_velocity_kd = 0;
			public static final double elevator_velocity_kf = 0;
			public static final int elevator_velocity_izone = 2;
			public static final double elevator_max_velocity_integral = 0.5;

			public static final double elevator_position_kp = 1;
			public static final double elevator_position_ki = 0;
			public static final double elevator_position_kd = 0;
			public static final double elevator_position_kf = 0.15;
			public static final int elevator_position_izone = 5000;
			public static final double elevator_max_position_integral = 5000;
		}

		public class elevatorTolerences {
			public static final double position_tolerence = 2;// Units are in inches
			public static final double velocity_tolerence = 0.5;// Units are in inches per second
		}
	}

	public static class intake {
		public static final int left_intake_talon_port = 6;
		public static final int right_intake_talon_port = 7;

		public class dimensions {
			//TODO add different dimensions of the intake, preferably WITHOUT having 17000 of them
		}
	}

	public static class pneumatics {
		// pheumatic configuration
		public static final int pcm_module_number = 9;
		public static final int drivetrain_solenoid_low_gear_channel = 7;
		public static final int drivetrain_solenoid_high_gear_channel = 3;
		public static final int intake_solenoid_clamp_channel = 0;
		public static final int intake_solenoid_open_channel = 6;
		public static final String drivetrain_starting_gear = "low";
	}

	/*
	public static class logging {
	public static final String default_filepath = "/home/lvuser/";
	public static final String[] data_headers = {
	"time", "left position", "left velocity", 
	"target left velocity", "target left voltage", 
	"right position", "right velocity", "target right velocity", "target right voltage",
	"gyro heading", "gyro rate of change", "robot voltage", "xbox forward axis", "xbox turn axis", "intake speed"
	};
	}
	*/
}
