package frc.robot.subsystems;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotConfig;
// import frc.robot.RobotConfig.driveTrain;
import frc.robot.commands.ArcadeDrive;
// import frc.robot.commands.stick_drive;
import frc.robot.lib.EncoderLib;

// import frc.robot.commands.drivetrain_shift_high;
// import frc.robot.commands.drivetrain_shift_low;

/**
 * Drivetrain subsystem. Initilizes the 4 drivetrain talons based on robotconfig
 * settings. Also, some motion profile stuff (which is hopefully depricated).
 * Contains methods for setting PID values, shifting, getting drivetrain encoder
 * status, setting speeds and voltages, and the arcade drive method.
 * 
 * @author Matthew Morley
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX m_left_talon = new TalonSRX(RobotConfig.driveTrain.leftTalons.m_left_talon_port);
  public TalonSRX s_left_talon = new TalonSRX(RobotConfig.driveTrain.leftTalons.s_left_talon_port);
  public TalonSRX m_right_talon = new TalonSRX(RobotConfig.driveTrain.rightTalons.m_right_talon_port);
  public TalonSRX s_right_talon = new TalonSRX(RobotConfig.driveTrain.rightTalons.s_right_talon_port);
  public String current_gear;
  public double tVoltage_l, tVoltage_r, tRaw_l, tRaw_r;

  public MotionProfileStatus m_left_MP_Status = new MotionProfileStatus();
  public MotionProfileStatus m_right_MP_Status = new MotionProfileStatus();

  public enum Gear {
    LOW, HIGH;
  }

  Gear gear;

  public void init() {
    m_left_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    m_right_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    s_left_talon.set(ControlMode.Follower, m_left_talon.getDeviceID());
    s_right_talon.set(ControlMode.Follower, m_right_talon.getDeviceID());
    m_right_talon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
    // Quadrature Encoder of current
    // Talon
    m_left_talon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
    // Quadrature Encoder of current
    // Talon
    m_left_talon.configPeakOutputForward(+1.0, 30);
    m_left_talon.configPeakOutputReverse(-1.0, 30);
    m_right_talon.configPeakOutputForward(+1.0, 30);
    m_right_talon.configPeakOutputReverse(-1.0, 30);

    m_left_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
    m_right_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);

    m_left_talon.setInverted(true);
    s_left_talon.setInverted(true);

    zeroEncoders();
    setHighGear();
  }

  public void setHighGear() {
    this.m_left_talon.config_kP(0, RobotConfig.driveTrain.leftTalons.velocity_kp_high, 30);
    this.m_left_talon.config_kI(0, RobotConfig.driveTrain.leftTalons.velocity_ki_high, 30);
    this.m_left_talon.config_kD(0, RobotConfig.driveTrain.leftTalons.velocity_kd_high, 30);
    this.m_left_talon.config_kF(0, RobotConfig.driveTrain.leftTalons.velocity_kf_high, 30);
    this.m_left_talon.config_IntegralZone(0, RobotConfig.driveTrain.leftTalons.velocity_izone_high, 30);
    // this.m_left_talon.configMaxIntegralAccumulator(0,
    // RobotConfig.driveTrain.left_talons.velocity_max_integral_high, 0);

    this.m_right_talon.config_kP(0, RobotConfig.driveTrain.rightTalons.velocity_kp_high, 30);
    this.m_right_talon.config_kI(0, RobotConfig.driveTrain.leftTalons.velocity_ki_high, 30);
    this.m_right_talon.config_kD(0, RobotConfig.driveTrain.leftTalons.velocity_kd_high, 30);
    this.m_right_talon.config_kF(0, RobotConfig.driveTrain.leftTalons.velocity_kf_high, 30);
    this.m_right_talon.config_IntegralZone(0, RobotConfig.driveTrain.leftTalons.velocity_izone_high, 30);
    // this.m_right_talon.configMaxIntegralAccumulator(0,
    // RobotConfig.m_right_velocity_max_integral_high, 0);

    // Trigger solenoids
    Robot.drivetrain_shift_high();
    gear = Gear.HIGH;
  }

  public void setLowGear() {
    this.m_left_talon.config_kP(0, RobotConfig.driveTrain.leftTalons.velocity_kp_low, 0);
    this.m_left_talon.config_kI(0, RobotConfig.driveTrain.leftTalons.velocity_ki_low, 0);
    this.m_left_talon.config_kD(0, RobotConfig.driveTrain.leftTalons.velocity_kd_low, 0);
    this.m_left_talon.config_kF(0, RobotConfig.driveTrain.leftTalons.velocity_kf_low, 0);
    this.m_left_talon.config_IntegralZone(0, RobotConfig.driveTrain.leftTalons.velocity_izone_low, 0);
    // this.m_left_talon.configMaxIntegralAccumulator(0,
    // RobotConfig.driveTrain.left_talons.velocity_max_integral_low, 0);

    this.m_right_talon.config_kP(0, RobotConfig.driveTrain.rightTalons.velocity_kp_low, 0);
    this.m_right_talon.config_kI(0, RobotConfig.driveTrain.rightTalons.velocity_ki_low, 0);
    this.m_right_talon.config_kD(0, RobotConfig.driveTrain.rightTalons.velocity_kd_low, 0);
    this.m_right_talon.config_kF(0, RobotConfig.driveTrain.rightTalons.velocity_kf_low, 0);
    this.m_right_talon.config_IntegralZone(0, RobotConfig.driveTrain.rightTalons.velocity_izone_low, 0);
    // this.m_right_talon.configMaxIntegralAccumulator(0,
    // RobotConfig.m_right_velocity_max_integral_low, 0);

    // Trigger solenoids
    Robot.drivetrain_shift_low();

    gear = Gear.LOW;
  }

  /** Return the current distance in feet of the left wheels */
  public double getLeftDistance() {
    return EncoderLib.rawToDistance(
      this.m_left_talon.getSelectedSensorPosition(0),
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
      RobotConfig.driveTrain.left_wheel_effective_diameter / 12
    );
  }

  public double getRightDistance() {
    return EncoderLib.rawToDistance(
      this.m_right_talon.getSelectedSensorPosition(0),
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
      RobotConfig.driveTrain.right_wheel_effective_diameter
    ) / 12;
  }

  public double getLeftVelocity() {
    return EncoderLib.rawToDistance(
      this.m_left_talon.getSelectedSensorVelocity(0) * 10,
      // Mulitply by 10 because units
      // are per 100ms
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
      RobotConfig.driveTrain.left_wheel_effective_diameter
    );
  }

  public double getRightVelocity() {
    return EncoderLib.rawToDistance(
      this.m_right_talon.getSelectedSensorVelocity(0) * 10,
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
      RobotConfig.driveTrain.right_wheel_effective_diameter
    );
  }

  public void zeroEncoders() {
    m_left_talon.setSelectedSensorPosition(0, 0, 30);
    m_right_talon.setSelectedSensorPosition(0, 0, 30);
  }

  /**
   * Set the drivetrain target speed as two doubles. For all ye lazy programmers
   * 
   * @param speed_left_raw  raw left speed
   * @param speed_right_raw raw right speed
   */
  public void setSpeeds(double speed_left_raw, double speed_right_raw) {
    setLeftSpeedRaw(speed_left_raw);
    setRightSpeedRaw(speed_right_raw);

    // Record the data for the logger to grab
    tRaw_l = speed_left_raw;
    tRaw_r = speed_right_raw;
    tVoltage_l = tVoltage_r = 0;
  }

  /**
   * An even more lazy version of @link setSpeeds This will literally set the
   * voltage of the left and right talons (from -1 to 1 ofc, like normal)
   * 
   * @param left_voltage
   * @param right_voltage
   */
  public void setVoltages(double left_voltage, double right_voltage) {
    m_left_talon.set(ControlMode.PercentOutput, left_voltage / 11);
    m_right_talon.set(ControlMode.PercentOutput, right_voltage / 11);

    // Record the data for the logger to grab
    tVoltage_l = left_voltage;
    tVoltage_r = right_voltage;
    tRaw_l = tRaw_r = 0;
  }

  /**
   * An even more lazy version of @link setSpeeds This will literally set the
   * throttle of the left and right talons (from -1 to 1 ofc, like normal)
   * 
   * @param leftspeed
   * @param rightspeed
   */
  public void setPowers(double left_power, double right_power) {
    m_left_talon.set(ControlMode.PercentOutput, left_power);
    m_right_talon.set(ControlMode.PercentOutput, right_power);
  }

  /**
   * Set the target left speed. Units are in raw units.
   * 
   * @param speed in raw units per 100ms
   */
  public void setLeftSpeedRaw(double speed) {
    m_left_talon.set(ControlMode.Velocity, speed);
  }

  /**
   * Set the target right speed. Units are in raw units.
   * 
   * @param speed in raw units per 100ms
   */
  public void setRightSpeedRaw(double speed) {
    m_right_talon.set(ControlMode.Velocity, speed);
  }

  public void arcadeDriveMethod(double forwardspeed, double turnspeed) {
    // double forwardspeed = Robot.m_oi.getForwardAxis() * -1;
    // double turnspeed = Robot.m_oi.getTurnAxis();

    if ((forwardspeed < 0.02) && (forwardspeed > -0.02)) {
      forwardspeed = 0;
    }
    if ((turnspeed < 0.01) && (turnspeed > -0.01)) {
      turnspeed = 0;
    }

    if (RobotConfig.controls.driving_squared) {
      forwardspeed = forwardspeed * Math.abs(forwardspeed);
      turnspeed = turnspeed * Math.abs(turnspeed);
    }
    if (Robot.drivetrain.gear == Gear.HIGH) {
      forwardspeed = forwardspeed * RobotConfig.driveTrain.max_forward_speed_high;
      turnspeed = turnspeed * RobotConfig.driveTrain.max_turn_speed_high;
    }
    if (Robot.drivetrain.gear == Gear.LOW) {
      forwardspeed = forwardspeed * RobotConfig.driveTrain.max_forward_speed_low;
      turnspeed = turnspeed * RobotConfig.driveTrain.max_turn_speed_low;
    }

    double leftspeed = forwardspeed + turnspeed; // units are in feet
    double rightspeed = forwardspeed - turnspeed;

    /**
     * Set left speed raw in feet per 100ms
     */
    double leftspeedraw = EncoderLib.distanceToRaw(
      leftspeed,
      RobotConfig.driveTrain.left_wheel_effective_diameter / 12,
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION
    ) / 10;
    // ((leftspeed) / (Math.PI *
    // RobotConfig.driveTrain.left_wheel_effective_diameter
    // / 12)) *
    // RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION
    // / 10;
    // divide by 10 becuase the talons want units per 100ms
    double rightspeedraw = EncoderLib.distanceToRaw(
      rightspeed,
      RobotConfig.driveTrain.right_wheel_effective_diameter / 12,
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION
    ) / 10;
    SmartDashboard.putNumber("left speed target", leftspeedraw);
    SmartDashboard.putNumber("right speed target", rightspeedraw);
    // TODO SmartDashboard code should NOT be placed in DriveTrain
    // Maybe move to Robot
    setLeftSpeedRaw(leftspeedraw);
    setRightSpeedRaw(rightspeedraw);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ArcadeDrive());
    // setDefaultCommand(new auto_action_DRIVE(5, "high", 5, 30));
  }
}
