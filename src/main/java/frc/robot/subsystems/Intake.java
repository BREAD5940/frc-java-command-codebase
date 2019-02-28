package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.subsystems.superstructure.IntakeTelop;

/**
 * The intake subsystem. Contains method setSpeed, openClamp and closeClamp.
 * Pretty barebones.
 * 
 * @author Matthew Morley
 */
public class Intake extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private static volatile Intake instance;
	private static Object mutex = new Object();

	public static Intake getInstance() {
		Intake result = instance;
		if (result == null) {
			synchronized (mutex) {
				result = instance;
				if (result == null)
					instance = result = new Intake();
			}
		}
		return result;
	}

	private DoubleSolenoid mSolenoid = Robot.getIntakeSolenoidInstance();

	private DoubleSolenoid getSolenoid() {
		return Robot.getIntakeSolenoidInstance();
	}

	public WPI_TalonSRX talon;
	// public TalonSRX talon_right = new TalonSRX(RobotConfig.intake.right_intake_talon_port);

	public enum HatchMechState {
		kOpen, kClamped;

		public static Value get(HatchMechState state) {
			return (state == kOpen) ? Value.kReverse : Value.kForward; // TODO check kforward state
		}
	}

	private HatchMechState hatchMechState;
	private static final HatchMechState kDefaultState = HatchMechState.kOpen;

	public HatchMechState getHatchMechState() {
		return (getSolenoid().get() == Value.kReverse) ? HatchMechState.kOpen : HatchMechState.kClamped; // TODO check kforward state
	}

	public void setHatchMech(HatchMechState mReq) {
		getSolenoid().set(HatchMechState.get(mReq));
	}

	float position_setpoint;

	private Intake(int port) {
		talon = new WPI_TalonSRX(port);
		// talon.configOpenloopRamp(0.15);
		talon.configContinuousCurrentLimit(30);
		talon.configPeakCurrentLimit(40);
		talon.enableCurrentLimit(true);
		talon.setName("Intake");
	}

	private Intake() {
		this(34);
	}

	/**
	 * Set speed to raw percent output
	 * @param speed
	 */
	public void setSpeed(double speed) {
		// if (Math.abs(speed) < 0.2)
		// 	speed = 0.1;
		// speed = Util.limit(speed, -0.75, 0.75);

		talon.set(ControlMode.PercentOutput, speed);
		SmartDashboard.putNumber("Intake speed setpoint", speed);

	}

	@Override
	public void periodic() {
		// setSpeed(Robot.m_oi.getIntakeSpeed());
		// System.out.println("speed " + Robot.m_oi.getIntakeSpeed());
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new IntakeTelop());
	}
}
