/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.robotconfig;

/**
 * Elevator subsystem
 */
public class elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  encoderlib encoderlib = new encoderlib();
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
    elevator_talon.setSelectedSensorPosition(0, 0, 0); // set position to zero. Run init at start of match to do this
  }

  // TODO methods for setting height based on nested PID

  /**
   * Returns the current height of the elevator in inches (same unit as effective radius)
   * @return height in inches
   */
  public double getHeight() {
    return encoderlib.rawToDistance(elevator_talon.getSelectedSensorPosition(0), robotconfig.POSITION_PULSES_PER_ROTATION, robotconfig.elevator_effective_radius);
  }





  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
