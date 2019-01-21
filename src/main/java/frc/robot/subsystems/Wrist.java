package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.wrist;
import frc.robot.lib.EncoderLib;


/**
 * A wrist subsystem with Talon hardware PID
 * Was never tested, the 775s burned out
 * 
 * @author Matthew Morley
 */
public class Wrist extends Subsystem  {

  public TalonSRX m_wrist_talon = new TalonSRX(wrist.m_wrist_talon_port);
  // private TalonSRX s_wrist_talon = new TalonSRX(wrist.s_wrist_talon_port);

  // double kf = wrist.talonConfig.software_position_kf;


  public Wrist() {
    // The constructor passes a name for the subsystem and the P, I and D constants that are used when computing the motor output
    // super("Wrist",  RobotConfig.wrist.talonConfig.software_position_kp, 
    //   RobotConfig.wrist.talonConfig.software_position_ki, 
    //   RobotConfig.wrist.talonConfig.software_position_kd);
		// setAbsoluteTolerance(0.05);
		// getPIDController().setContinuous(false);
  }

  public void init() {
    // setup the mag encoder
    m_wrist_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    // s_wrist_talon.set(ControlMode.Follower, s_wrist_talon.getDeviceID());

    m_wrist_talon.setInverted(wrist.talon_direction_inverted);
    m_wrist_talon.setSensorPhase(wrist.talon_encoder_inverted);

    // Quadrature Encoder of current Talon
    m_wrist_talon.configPeakOutputForward(+1.0, 30);
    m_wrist_talon.configPeakOutputReverse(-1.0, 30);
    m_wrist_talon.setSelectedSensorPosition(0, 0, 10);
    // WARNING TODO so if the robot calls init(), the wrist will zero itself. Is this the behavior we want? Limit switches? 

    // configure PID
    m_wrist_talon.config_kP(0, wrist.talonConfig.position_kp, 0);
    m_wrist_talon.config_kI(0, wrist.talonConfig.position_ki, 0);
    m_wrist_talon.config_kD(0, wrist.talonConfig.position_kd, 0);
    m_wrist_talon.config_kF(0, wrist.talonConfig.gravity_ff, 0);
    m_wrist_talon.config_IntegralZone(0, wrist.talonConfig.position_izone, 0);
    // m_wrist_talon.configMaxIntegralAccumulator(0, RobotConfig.m_left_velocity_max_integral_low, 0);
  }

  // @Override
  // protected double returnPIDInput() {
  //   return getAngle(); // returns the sensor value that is providing the feedback for the system
  // }

  // @Override
  // protected void usePIDOutput(double output) {
  //   m_wrist_talon.set(ControlMode.PercentOutput, (output + (Math.abs(kf * Math.cos(output)))) / 11 );
  //   // this is where the computed output value fromthe PIDController is applied to the motor
  // }
  
  /**
   * Get the raw position of the wrist
   * @return degrees of the wrist
   */
  public double getAngle(){
    return EncoderLib.rawToDegrees(
      m_wrist_talon.getSelectedSensorPosition(0), 
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);
  }
  public double getAngularVelocity(){
    return EncoderLib.rawToDegrees(
      m_wrist_talon.getSelectedSensorVelocity(0), 
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) * 10;
      // Angular velocity. Natively is raw per 100ms, so times by 10 to get degrees per second
  }

  public void setNeutralMode(NeutralMode mode) {
    m_wrist_talon.setNeutralMode(mode);
  }

  public void setMotioMmagic(double angle) {
    m_wrist_talon.set(ControlMode.MotionMagic, angle, DemandType.ArbitraryFeedForward, 
    wrist.talonConfig.gravity_ff * Math.abs(Math.cos( Math.toRadians(getAngle()))));
  }

  public void setPosition(double angle) {
    m_wrist_talon.set(ControlMode.Position, angle, DemandType.ArbitraryFeedForward, 
    wrist.talonConfig.gravity_ff * Math.abs(Math.cos( Math.toRadians(getAngle()))));
  }

  /**
   * Return if the wrist is within a specified target angle
   */
  public boolean isWithinTolerence(double target) {
    return ( Math.abs(target - getAngle()) < RobotConfig.wrist.wrist_position_tolerence );
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
