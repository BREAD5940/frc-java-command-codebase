/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.robotconfig;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * Elevator subsystem
 */
public class elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX elevator_talon = new TalonSRX(robotconfig.elevator_talon_port);
  float position_setpoint;

  public elevator() {
    super("elevator");
  }

  public void init() {
    elevator_talon.config_kP(0, robotconfig.elevator_velocity_kp, 10);
    elevator_talon.config_kI(0, robotconfig.elevator_velocity_ki, 10);
    elevator_talon.config_kD(0, robotconfig.elevator_velocity_kd, 10);
    elevator_talon.config_kF(0, robotconfig.elevator_velocity_kf, 10);
    elevator_talon.config_IntegralZone(0, robotconfig.elevator_velocity_izone, 10);
    elevator_talon.configMaxIntegralAccumulator(0, robotconfig.elevator_max_velocity_integral, 10);
    elevator_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0); 
    elevator_talon.setSensorPhase(true);
		elevator_talon.configVoltageCompSaturation(12, 0);
		elevator_talon.enableVoltageCompensation(true);
    elevator_talon.setSelectedSensorPosition(0, 0, 0);
  }

  // TODO methods for setting height based on nested PID


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
