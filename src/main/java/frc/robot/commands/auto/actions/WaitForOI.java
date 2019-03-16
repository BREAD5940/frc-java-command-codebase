package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

public class WaitForOI extends Command {
	Command toRun;
	Button toPoll;
	public WaitForOI(Command toRunWhileWaiting, Button toPoll) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.toRun = toRunWhileWaiting;
		this.toPoll = toPoll;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		this.clearRequirements();
		toRun.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return toPoll.get();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("======== button pressed! ========");
		toRun.cancel();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
