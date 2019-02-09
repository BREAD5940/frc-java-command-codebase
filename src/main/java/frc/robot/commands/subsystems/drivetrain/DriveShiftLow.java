package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.Logger;

/**
 * Shifter command to shift to low gear
 */
public class DriveShiftLow extends Command {
	public DriveShiftLow() {
		// Use requires() here to declare subsystem dependencies
		// requires(Robot.drivetrain); // Not really necessary.... commented out
	}

	// public static final drivetrain drivetrain = new drivetrain();

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Logger.log("setting low gear");
		Robot.drivetrain.setLowGear();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
