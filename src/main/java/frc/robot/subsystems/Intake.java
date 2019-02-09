package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * The intake subsystem. Contains method setSpeed, openClamp and closeClamp.
 * Pretty barebones.
 * 
 * @author Matthew Morley
 */
public class Intake extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public WPI_TalonSRX talon = new WPI_TalonSRX(11);
	// public TalonSRX talon_right = new TalonSRX(RobotConfig.intake.right_intake_talon_port);

	float position_setpoint;

	/**
	 * Set speed to raw percent output
	 * @param speed
	 */
	public void setSpeed(double speed) {
		talon.set(ControlMode.PercentOutput, speed);
		SmartDashboard.putNumber("Intake speed setpoint", speed);
	}

	@Override
	public void periodic() {
		setSpeed(Robot.m_oi.getIntakeSpeed());
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new IntakeTelop());
	}
}
