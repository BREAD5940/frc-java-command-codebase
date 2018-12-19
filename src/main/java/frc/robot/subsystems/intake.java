package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.robotconfig;
import frc.robot.commands.run_intake;

// import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * The intake subsystem. Contains method setSpeed, openClamp and closeClamp
 */
public class intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX talon_left = new TalonSRX(robotconfig.left_intake_talon_port);
  public TalonSRX talon_right = new TalonSRX(robotconfig.right_intake_talon_port);

  // public DoubleSolenoid intake_solenoid = new DoubleSolenoid(robotconfig.intake_solenoid_clamp_channel, robotconfig.intake_solenoid_open_channel);
  float position_setpoint;

/**
 * Set speed to raw percent output
 * @param double speed
 */
  public void setSpeed(double speed) {
    talon_left.set(ControlMode.PercentOutput, speed);
    talon_right.set(ControlMode.PercentOutput, speed);
  }

  public void openClamp() {
    // solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void closeClamp() {
    // solenoid.set(DoubleSolenoid.Value.kForward);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new run_intake());
  }
}
