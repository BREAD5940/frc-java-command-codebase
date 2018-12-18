package frc.robot;
public class robotconfig {

  /**
   * Joystick configuration
   */
  public static final int primary_joystick_port = 0;
  public static final int forward_axis = xboxmap.Axis.LEFT_JOYSTICK_Y;
  public static final int turn_axis = xboxmap.Axis.RIGHT_JOYSTICK_X;
  public static final int shift_down_button = xboxmap.Buttons.LB_BUTTON;
  public static final int shift_up_button = xboxmap.Buttons.RB_BUTTON;
  public static final int intakeAxis = xboxmap.Axis.RIGHT_TRIGGER;
  public static final int outtakeAxis = xboxmap.Axis.LEFT_TRIGGER;
  public static final int intakeOpen = xboxmap.Buttons.LB_BUTTON;
  public static final int intakeClose = xboxmap.Buttons.RB_BUTTON;
  public static final int secondary_joystick_port = 1;
  public static final int throttle_axis = 2;

  /**
   * Wrist configuration
   */
  public static final int m_wrist_talon_port = 5; // TODO fix this
  public static final int s_wrist_talon_port = 4; // TODO fix this
  public static final float minimum_wrist_angle = 0;
  public static final float maximum_wrist_angle = 90;
  public static final double wrist_position_tolerence = 5; // 5 degree tolerence, be sure to convert to raw!
  public static final double wrist_velocity_tolerence = 2; // 2 degrees per second??
  public static final double wrist_position_kp_low = 0.1;
  public static final double wrist_position_ki_low = 0;
  public static final double wrist_position_kd_low = 0;
  public static final double wrist_position_kf_low = 0; // TODO feed forward math
  public static final int wrist_position_izone_low = 300;
  // public static final double wrist_position_max_integral_low = 1;

  /**
   * Robot configuration
   */
  // TODO make this count towards drivetrain PID (as a cap on velocity?)
  public static final float max_forward_speed = 5; // Feet per second forward velocity
  public static final float max_turn_speed = 5; // Max turn speed in degrees per second

  // Encoder stuff, dunno where else to put this
	// public static final double VELOCITY_PULSES_PER_ROTATION = 409.6f;
  public static final double POSITION_PULSES_PER_ROTATION = 4096f;
  

  public static final double elevator_effective_diameter = 1; // TODO fix this!!!! (units must be inches)
  public static final int elevator_minimum_height = 0;
  // public static final int elevator_maximum_height = 42000; // TODO fix this!!!!
  public static final int elevator_maximum_height = 70; // changed to inches

  /**
   * Left side of drivetrain PID constants and setup
   */
  public static final int m_left_talon_port = 2;
  public static final int s_left_talon_port = 1;
  public static final boolean m_left_inverted = false;
  // sets kp, ki, kd and kf terms for master left in velocity mode 
  public static final double m_left_velocity_kp_low = 0.08;
  public static final double m_left_velocity_ki_low = 0.03;
  public static final double m_left_velocity_kd_low = 0;
  public static final double m_left_velocity_kf_low =  0;
  public static final int m_left_velocity_izone_low = 300;
  // public static final double m_left_velocity_max_integral_low = 500000;
  public static final double m_left_position_kp_low = 0.1;
  public static final double m_left_position_ki_low = 0;
  public static final double m_left_position_kd_low = 20;
  public static final double m_left_position_kf_low = 0.15;
  public static final int m_left_position_izone_low = 300;
  // public static final double m_left_position_max_integral_low = 1;
  public static final double m_left_velocity_kp_high = 0.1;
  public static final double m_left_velocity_ki_high = 0.02;
  public static final double m_left_velocity_kd_high = 0;
  public static final double m_left_velocity_kf_high = 0.05;
  public static final int m_left_velocity_izone_high = 300;
  // public static final double m_left_velocity_max_integral_high = 300;
  public static final double m_left_position_kp_high = 0.15;
  public static final double m_left_position_ki_high = 0;
  public static final double m_left_position_kd_high = 20;
  public static final double m_left_position_kf_high = 0.1;
  public static final int m_left_position_izone_high = 300;
  // public static final double m_left_position_max_integral_high = 5;

  /**
   * Right side of drivetrain PID constants and setup
   */
  public static final int m_right_talon_port = 4;
  public static final int s_right_talon_port = 3;
  public static final double m_right_velocity_kp_low = m_left_velocity_kp_low;
  public static final double m_right_velocity_ki_low = m_left_velocity_ki_low;
  public static final double m_right_velocity_kd_low = m_left_velocity_kd_low;
  public static final double m_right_velocity_kf_low = m_left_velocity_kf_low;
  public static final int m_right_velocity_izone_low = m_left_velocity_izone_low;
  public static final double m_right_velocity_max_integral_low = 5;
  public static final double m_right_position_kp_low = m_left_position_kp_low;
  public static final double m_right_position_ki_low = m_left_position_ki_low;
  public static final double m_right_position_kd_low = m_left_position_kd_low;
  public static final double m_right_position_kf_low = m_left_position_kf_low;
  // public static final int m_left_position_izone_low = 5;
  public static final double m_right_position_max_integral_low = 1;
  public static final double m_right_velocity_kp_high = m_left_velocity_kp_high;
  public static final double m_right_velocity_ki_high = m_left_velocity_ki_high;
  public static final double m_right_velocity_kd_high = m_left_velocity_kd_high;
  public static final double m_right_velocity_kf_high = m_left_velocity_kf_high;
  public static final int m_right_velocity_izone_high = m_left_velocity_izone_high;
  public static final double m_right_velocity_max_integral_high = 5;
  public static final double m_right_position_kp_high = m_left_position_kp_high;
  public static final double m_right_position_ki_high = m_left_position_ki_high;
  public static final double m_right_position_kd_high = m_left_position_kd_high;
  public static final double m_right_position_kf_high = m_left_position_kf_high;
  // public static final int m_right_position_izone_high = 5;
  public static final double m_right_position_max_integral_high = 5;

  /**
   * Elevator configuration
   */
  public static final int elevator_talon_port = 5;
  public static final int left_intake_talon_port = 6;
  public static final int right_intake_talon_port = 7;
  public static final int intake_solenoid_clamp_channel = 0;
  public static final int intake_solenoid_open_channel = 6;

  public static final double elevator_velocity_kp = 1;
  public static final double elevator_velocity_ki = 0;
  public static final double elevator_velocity_kd = 0;
  public static final double elevator_velocity_kf = 0;
  public static final int elevator_velocity_izone = 2;
  public static final double elevator_max_velocity_integral = 0.5;

  public static final double elevator_position_kp = 1.7;
  public static final double elevator_position_ki = 0;
  public static final double elevator_position_kd = 0;
  public static final double elevator_position_kf = 0;
  public static final int elevator_position_izone = 2;
  public static final double elevator_max_position_integral = 0.5;


  // pheumatic configuration
  public static final int pcm_module_number = 9;
  public static final int drivetrain_solenoid_low_gear_channel = 7;
  public static final int drivetrain_solenoid_high_gear_channel = 3;
  public static final String drivetrain_starting_gear = "low";

}