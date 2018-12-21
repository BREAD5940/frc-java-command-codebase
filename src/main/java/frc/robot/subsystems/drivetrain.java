package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.robot.Robot;
import frc.robot.lib.EncoderLib;
import frc.robot.RobotConfig;
import frc.robot.auto.auto_action_DRIVE;
import frc.robot.commands.ArcadeDrive;
// import frc.robot.commands.stick_drive;

// import frc.robot.commands.drivetrain_shift_high;
// import frc.robot.commands.drivetrain_shift_low;


/**
 * Drivetrain subsystem. Initilizes the 4 drivetrain talons based on robotmap
 * settings
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

    // Robot Robot = new Robot();

    public TalonSRX m_left_talon = new TalonSRX(RobotConfig.m_left_talon_port);
    public TalonSRX s_left_talon = new TalonSRX(RobotConfig.s_left_talon_port);
    public TalonSRX m_right_talon = new TalonSRX(RobotConfig.m_right_talon_port);
    public TalonSRX s_right_talon = new TalonSRX(RobotConfig.s_right_talon_port);
    public String current_gear;


    // PIDController left_position_pid_controller_HIGH_GEAR = new PIDController(RobotConfig.m_left_position_kp_high, RobotConfig.m_left_position_ki_high, RobotConfig.m_left_position_kd_high, RobotConfig.m_left_position_kf_high, source, output) // TODO Implament position PID controllers for left and right drivetrain sides



    // Robot robot = new Robot(); 
    

    public void init() {
      m_left_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30); // TODO put encoder stats on smartdashboard
      m_right_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30); // TODO put encoder stats on smartdashboard
      s_left_talon.set(ControlMode.Follower, m_left_talon.getDeviceID());
      s_right_talon.set(ControlMode.Follower, m_right_talon.getDeviceID());
      m_right_talon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30); // Quadrature Encoder of current Talon
      m_left_talon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30); // Quadrature Encoder of current Talon

      m_left_talon.configPeakOutputForward(+1.0, 30);
      m_left_talon.configPeakOutputReverse(-1.0, 30);
      m_right_talon.configPeakOutputForward(+1.0, 30);
      m_right_talon.configPeakOutputReverse(-1.0, 30);


    	/* 1ms per loop.  PID loop can be slowed down if need be.
      * For example,
      * - if sensor updates are too slow
      * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
      * - sensor movement is very slow causing the derivative error to be near zero.
      */
      int closedLoopTimeMs = 1;
      // TODO can I get rid of this code, or is it disabling something important?
      m_right_talon.configSetParameter(ParamEnum.eSampleVelocityPeriod, closedLoopTimeMs, 0x00, 0,30);
      m_right_talon.configSetParameter(ParamEnum.eSampleVelocityPeriod, closedLoopTimeMs, 0x00, 1, 30);
      
      m_left_talon.setInverted(true);
      s_left_talon.setInverted(true);

      m_left_talon.setSelectedSensorPosition(0, 0, 30);
      m_right_talon.setSelectedSensorPosition(0, 0, 30);

      // public String gear_state;

      // Robot.shifter.set(DoubleSolenoid.Value.kForward);
      // Robot.shifter.set(DoubleSolenoid.Value.kReverse);

      setHighGear();

      // if ( RobotConfig.drivetrain_starting_gear == "low" ) {
      //   new drivetrain_shift_low();
      // }
      // if ( RobotConfig.drivetrain_starting_gear == "high" ) {
      //   new drivetrain_shift_high();
      // }

    }
    


    public void setHighGear() {
      this.m_left_talon.config_kP(0, RobotConfig.m_left_velocity_kp_high, 30);
      this.m_left_talon.config_kI(0, RobotConfig.m_left_velocity_ki_high, 30);
      this.m_left_talon.config_kD(0, RobotConfig.m_left_velocity_kd_high, 30);
      this.m_left_talon.config_kF(0, RobotConfig.m_left_velocity_kf_high, 30);
      this.m_left_talon.config_IntegralZone(0, RobotConfig.m_left_velocity_izone_high, 30);
      // this.m_left_talon.configMaxIntegralAccumulator(0, RobotConfig.m_left_velocity_max_integral_high, 0);
  
      this.m_right_talon.config_kP(0, RobotConfig.m_right_velocity_kp_high, 30);
      this.m_right_talon.config_kI(0, RobotConfig.m_right_velocity_ki_high, 30);
      this.m_right_talon.config_kD(0, RobotConfig.m_right_velocity_kd_high, 30);
      this.m_right_talon.config_kF(0, RobotConfig.m_right_velocity_kf_high, 30);
      this.m_right_talon.config_IntegralZone(0, RobotConfig.m_right_velocity_izone_high, 30);
      // this.m_right_talon.configMaxIntegralAccumulator(0, RobotConfig.m_right_velocity_max_integral_high, 0);
      
      // Trigger solenoids
      Robot.drivetrain_shift_high();
      current_gear = "high";
    }

    public void setLowGear() {
      this.m_left_talon.config_kP(0, RobotConfig.m_left_velocity_kp_low, 0);
      this.m_left_talon.config_kI(0, RobotConfig.m_left_velocity_ki_low, 0);
      this.m_left_talon.config_kD(0, RobotConfig.m_left_velocity_kd_low, 0);
      this.m_left_talon.config_kF(0, RobotConfig.m_left_velocity_kf_low, 0);
      this.m_left_talon.config_IntegralZone(0, RobotConfig.m_left_velocity_izone_low, 0);
      // this.m_left_talon.configMaxIntegralAccumulator(0, RobotConfig.m_left_velocity_max_integral_low, 0);
      
      this.m_right_talon.config_kP(0, RobotConfig.m_right_velocity_kp_low, 0);
      this.m_right_talon.config_kI(0, RobotConfig.m_right_velocity_ki_low, 0);
      this.m_right_talon.config_kD(0, RobotConfig.m_right_velocity_kd_low, 0);
      this.m_right_talon.config_kF(0, RobotConfig.m_right_velocity_kf_low, 0);
      this.m_right_talon.config_IntegralZone(0, RobotConfig.m_right_velocity_izone_low, 0);
      // this.m_right_talon.configMaxIntegralAccumulator(0, RobotConfig.m_right_velocity_max_integral_low, 0);
  
      // Trigger solenoids
      Robot.drivetrain_shift_low();

      current_gear = "low";
    }

  public double getLeftDistance() {return EncoderLib.rawToDistance(this.m_left_talon.getSelectedSensorPosition(0), 
    RobotConfig.POSITION_PULSES_PER_ROTATION, RobotConfig.left_wheel_effective_diameter / 12); }
  public double getRightDistance() {return EncoderLib.rawToDistance(this.m_right_talon.getSelectedSensorPosition(0), 
    RobotConfig.POSITION_PULSES_PER_ROTATION, RobotConfig.right_wheel_effective_diameter) / 12;}
  public double getLeftVelocity() {return EncoderLib.rawToDistance(this.m_left_talon.getSelectedSensorVelocity(0) * 10, //Mulitply by 10 because units are per 100ms 
    RobotConfig.POSITION_PULSES_PER_ROTATION, RobotConfig.left_wheel_effective_diameter);}
  public double getRightVelocity() {return EncoderLib.rawToDistance(this.m_right_talon.getSelectedSensorVelocity(0) * 10, 
    RobotConfig.POSITION_PULSES_PER_ROTATION, RobotConfig.right_wheel_effective_diameter);}

  /**
   * Set the target left speed. Units are in raw units.
   * @param speed in raw units per 100ms
   */
  public void setLeftSpeedRaw(double speed){
    m_left_talon.set(ControlMode.Velocity, speed);
  }
  /**
   * Set the target right speed. Units are in raw units.
   * @param speed in raw units per 100ms
   */
  public void setRightSpeedRaw(double speed){
    m_right_talon.set(ControlMode.Velocity, speed);
  }

  public void arcadeDriveMethod(double forwardspeed, double turnspeed){
    // double forwardspeed = Robot.m_oi.getForwardAxis() * -1;
    // double turnspeed = Robot.m_oi.getTurnAxis();


    if ((forwardspeed < 0.02) && (forwardspeed > -0.02)) { forwardspeed = 0; }
    if ((turnspeed < 0.01) && (turnspeed > -0.01)) { turnspeed = 0; }
    
    if (RobotConfig.driving_squared) {
      forwardspeed = forwardspeed * Math.abs(forwardspeed);
      turnspeed = turnspeed * Math.abs(turnspeed);
    }
    if (Robot.drivetrain.current_gear == "high"){
        forwardspeed = forwardspeed * RobotConfig.max_forward_speed_high;
        turnspeed = turnspeed * RobotConfig.max_turn_speed_high;}
    if (Robot.drivetrain.current_gear == "low"){
        forwardspeed = forwardspeed * RobotConfig.max_forward_speed_low;
        turnspeed = turnspeed * RobotConfig.max_turn_speed_low;}

    double leftspeed = forwardspeed + turnspeed; // units are in feet
    double rightspeed = forwardspeed - turnspeed;

    /**
     * Set left speed raw in feet per 100ms
     */
    double leftspeedraw = EncoderLib.distanceToRaw(leftspeed, RobotConfig.left_wheel_effective_diameter / 12, RobotConfig.POSITION_PULSES_PER_ROTATION) / 10;//  ((leftspeed) / (Math.PI * RobotConfig.left_wheel_effective_diameter / 12)) * RobotConfig.POSITION_PULSES_PER_ROTATION / 10;
    // divide by 10 becuase the talons want units per 100ms
    double rightspeedraw = EncoderLib.distanceToRaw(rightspeed, RobotConfig.right_wheel_effective_diameter / 12, RobotConfig.POSITION_PULSES_PER_ROTATION) / 10;

    setLeftSpeedRaw(leftspeedraw);
    setRightSpeedRaw(rightspeedraw);

  }

  /**
   * Shitty P loop. Literally a P loop. Super boring.
   * @param kp gain
   * @param setpoint
   * @param measured sensor measurement
   * @param minimum_output of the controller
   * @param maximum_output of the controller
   */
  public double shitty_P_loop(double kp, double setpoint, double measured, double minimim_output, double maximum_output) {
    double error = -measured + setpoint;
    double output = error * kp;
    if ( output > maximum_output ) { output = maximum_output; }
    if ( output < minimim_output ) { output = minimim_output; }
    return output;
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ArcadeDrive());
    // setDefaultCommand(new auto_action_DRIVE(5, "high", 5, 30));
  }
}
