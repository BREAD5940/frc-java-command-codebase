package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConfig;
import frc.robot.commands.RunIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * The intake subsystem. Contains method setSpeed, openClamp and closeClamp.
 * Pretty barebones.
 * 
 * @author Matthew Morley
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX talon_left = new TalonSRX(RobotConfig.intake.left_intake_talon_port);
  public TalonSRX talon_right = new TalonSRX(RobotConfig.intake.right_intake_talon_port);

  float position_setpoint;

/**
 * Set speed to raw percent output
 * @param double speed
 */
  public void setSpeed(double speed) {
    talon_left.set(ControlMode.PercentOutput, speed);
    talon_right.set(ControlMode.PercentOutput, -speed);
    SmartDashboard.putNumber("Intake speed setpoint", speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new RunIntake());
  }
}
