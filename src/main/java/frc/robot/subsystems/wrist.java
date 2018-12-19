package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.robotconfig;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.lib.encoderlib;


/**
 * The intake subsystem. Contains method setSpeed, openClamp and closeClamp
 */
public class wrist extends Subsystem {

  public TalonSRX m_wrist_talon = new TalonSRX(robotconfig.m_wrist_talon_port);
  private TalonSRX s_wrist_talon = new TalonSRX(robotconfig.s_wrist_talon_port);

  public void init() {

    // setup the mag encoder
    this.m_wrist_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30); // TODO put encoder stats on smartdashboard
    this.s_wrist_talon.set(ControlMode.Follower, s_wrist_talon.getDeviceID());
    this.m_wrist_talon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30); // Quadrature Encoder of current Talon
    this.m_wrist_talon.configPeakOutputForward(+1.0, 30);
    this.m_wrist_talon.configPeakOutputReverse(-1.0, 30);
    this.m_wrist_talon.setSelectedSensorPosition(0, 0, 10); // WARNING TODO so if the robot calls init(), the wrist will zero itself. Is this the behavior we want? Limit switches? 

    // configure PID
    this.m_wrist_talon.config_kP(0, robotconfig.m_left_velocity_kp_low, 0);
    this.m_wrist_talon.config_kI(0, robotconfig.m_left_velocity_ki_low, 0);
    this.m_wrist_talon.config_kD(0, robotconfig.m_left_velocity_kd_low, 0);
    this.m_wrist_talon.config_kF(0, robotconfig.m_left_velocity_kf_low, 0);
    this.m_wrist_talon.config_IntegralZone(0, robotconfig.m_left_velocity_izone_low, 0);
    // this.m_wrist_talon.configMaxIntegralAccumulator(0, robotconfig.m_left_velocity_max_integral_low, 0);
  }
  
  public double getAngle(){
    return encoderlib.rawToDegrees(
      this.m_wrist_talon.getSelectedSensorPosition(0), 
      robotconfig.POSITION_PULSES_PER_ROTATION);
  }
  public double getAngularVelocity(){
    return encoderlib.rawToDegrees(
      this.m_wrist_talon.getSelectedSensorVelocity(0), 
      robotconfig.POSITION_PULSES_PER_ROTATION) * 10; // Angular velocity. Natively is raw per 100ms, so times by 10 to get degrees per second
  }

  public void setAngle(double target_angle){ // TODO verify math
    double targetRaw = encoderlib.degreesToRaw(
      target_angle,
      robotconfig.POSITION_PULSES_PER_ROTATION);
    m_wrist_talon.set(ControlMode.Position, targetRaw);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
