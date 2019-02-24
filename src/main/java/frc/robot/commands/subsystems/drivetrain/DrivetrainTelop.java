package frc.robot.commands.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Default drivetrain command. This *should* be called as the default drivetrain
 * command and be overridden in autononmous (provided auto requires
 * drivetrain???) This command uses the Robot.m_oi to set the speed based on
 * xbox controller inputs, arcade style
 * 
 * @author Matthew Morley
 */
public class DrivetrainTelop extends Command {

	// System.out.println("im an arcade drive command!");
	/**
	 * This command runs arcade drive as the default command for the drivetrain.
	 * This command will reserve the drivetrain.
	 */
	public DrivetrainTelop() {
		requires(Robot.drivetrain);
	}

	// drivetrain drivetrain = new drivetrain();

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
		Robot.drivetrain.getLeft().getMaster().configOpenloopRamp(0.1);
		Robot.drivetrain.getRight().getMaster().configOpenloopRamp(0.1);
		Robot.drivetrain.arcadeDrive(0, 0);
		System.out.println("arcade drive command init");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Robot.drivetrain.arcadeDrive(Robot.m_oi.getForwardAxis(),
		//   Robot.m_oi.getTurnAxis());

		boolean isQuickTurn = (Math.abs(Robot.m_oi.getForwardAxis()) < 0.08);
		// boolean isQuickTurn = (Math.abs(Robot.m_oi.getPrimary().getRawAxis(0)) > 0.75);

		Robot.drivetrain.arcadeDrive(Robot.m_oi.getForwardAxis() * 1,
				Robot.m_oi.getTurnAxis() * 1/*, isQuickTurn*/);

		// Logger.log("forward command: " + Robot.m_oi.getForwardAxis());

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// System.out.println("we aint done chief");
		return false;

	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivetrain.arcadeDrive(0, 0);
		System.out.println("arcade end called");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.drivetrain.arcadeDrive(0, 0);
	}
}
