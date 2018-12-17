package frc.robot;
public class robotconfig {


  // Joystick configuration
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

  // Robot configuration
  // TODO make this count towards drivetrain PID (as a cap on velocity?)
  public static final float max_forward_speed = 5; // Feet per second forward velocity
  public static final float max_turn_speed = 5; // Max turn speed in degrees per second

  // TalonSRX configuration
  public static final int drivetrain_solenoid_low_gear_channel = 1;
  public static final int drivetrain_solenoid_high_gear_channel = 2;
  public static final String drivetrain_starting_gear = "low";

  // Encoder stuff, dunno where else to put this
	public static final double VELOCITY_PULSES_PER_ROTATION = 409.6f;
	public static final double POSITION_PULSES_PER_ROTATION = 4096f;
  
  // Left talon setup
  public static final int m_left_talon_port = 1;
  public static final int s_left_talon_port = 2;
  public static final boolean m_left_inverted = false;

  // sets kp, ki, kd and kf terms for master left in velocity mode 
  public static final double m_left_velocity_kp_low = 1;
  public static final double m_left_velocity_ki_low = 0;
  public static final double m_left_velocity_kd_low = 0;
  public static final double m_left_velocity_kf_low = 0;
  public static final int m_left_velocity_izone_low = 5;
  public static final double m_left_velocity_max_integral_low = 5;

  public static final double m_left_position_kp_low = 1;
  public static final double m_left_position_ki_low = 1;
  public static final double m_left_position_kd_low = 1;
  public static final double m_left_position_kf_low = 1;
  // public static final int m_left_position_izone_low = 5;
  public static final double m_left_position_max_integral_low = 1;

  public static final double m_left_velocity_kp_high = 1;
  public static final double m_left_velocity_ki_high = 0;
  public static final double m_left_velocity_kd_high = 0;
  public static final double m_left_velocity_kf_high = 0;
  public static final int m_left_velocity_izone_high = 5;
  public static final double m_left_velocity_max_integral_high = 5;


  public static final double m_left_position_kp_high = 1;
  public static final double m_left_position_ki_high = 1;
  public static final double m_left_position_kd_high = 1;
  public static final double m_left_position_kf_high = 1;
  // public static final int m_left_position_izone_high = 5;
  public static final double m_left_position_max_integral_high = 5;



  public static final int m_right_talon_port = 3;
  public static final int s_right_talon_port = 4;
  public static final boolean m_right_inverted = false;

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


  public static final int elevator_talon_port = 3;
  public static final int intake_talon_port = 4;
  public static final int elevator_solenoid_clamp_channel = 1;
  public static final int elevator_solenoid_open_channel = 2;


  public static final double elevator_velocity_kp = 1;
  public static final double elevator_velocity_ki = 0;
  public static final double elevator_velocity_kd = 0;
  public static final double elevator_velocity_kf = 0;
  public static final int elevator_velocity_izone = 2;
  public static final double elevator_max_velocity_integral = 0.5;

  public static final double elevator_position_kp = 1;
  public static final double elevator_position_ki = 0;
  public static final double elevator_position_kd = 0;
  public static final double elevator_position_kf = 0;
  public static final int elevator_position_izone = 2;
  public static final double elevator_max_position_integral = 0.5;


}