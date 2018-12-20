package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;
import frc.robot.robotconfig;
import frc.robot.commands.elevator_teleop;
import frc.robot.lib.encoderlib;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


/**
 * The intake subsystem. Contains method setSpeed, openClamp and closeClamp
 */
public class elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX elevator_talon = new TalonSRX(robotconfig.elevator_talon_port);

  double raw_max_height = encoderlib.distanceToRaw(robotconfig.elevator_maximum_height, robotconfig.POSITION_PULSES_PER_ROTATION, robotconfig.elevator_effective_diameter);

  float position_setpoint;

/**
 * Set height to raise elevator to
 * @param double height
 */

  public void init(){
    elevator_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    elevator_talon.setSelectedSensorPosition(0, 0, 10); // zero the encoder
    elevator_talon.setInverted(false);
    elevator_talon.setSensorPhase(true);
    this.elevator_talon.config_kP(0, robotconfig.elevator_position_kp, 30);
    this.elevator_talon.config_kI(0, robotconfig.elevator_position_ki, 30);
    this.elevator_talon.config_kD(0, robotconfig.elevator_position_kd, 30);
    this.elevator_talon.config_kF(0, robotconfig.elevator_position_kf, 30);
    setHeight(Robot.elevator.getElevatorAxisInches());
  }

  /**
   * Get the current elevator height from the throttle in inches. For passing into setHeight
   * @return height in inches
   */
  public double getElevatorAxisInches(){
    return (Robot.m_oi.getThrottleAxis() / (robotconfig.throttle_maximum_value - robotconfig.throttle_minimum_value)) 
      * robotconfig.elevator_maximum_height;
  }


  /**
   * Set the elevator height, in inches
   * @param height in inches
   */
  public void setHeight(double height) {
    if(height>raw_max_height){
      height = raw_max_height;//reset to maximum if too high
    }else if (height<0){
      height = 0;
    }
      //  however you may want to consider catching errors like elevator below minimum height, or elevator above maximum height, and allowing the elevator to move
    elevator_talon.set(
      ControlMode.Position, encoderlib.distanceToRaw(
        height, 
        robotconfig.POSITION_PULSES_PER_ROTATION, 
        robotconfig.elevator_effective_diameter));
  }

  public void setPercent(double percent){
    elevator_talon.set(ControlMode.PercentOutput,percent);
  }

  /**
   * Return the height of the elevator from zero, in inches
   * @return height in inches
   */
  public double getHeight() {
    double inches = encoderlib.rawToDistance(
      elevator_talon.getSelectedSensorPosition(0), 
      robotconfig.POSITION_PULSES_PER_ROTATION, 
      robotconfig.elevator_effective_diameter);
    // return inches;
    return elevator_talon.getSelectedSensorPosition(0);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new elevator_teleop());
  }
}
