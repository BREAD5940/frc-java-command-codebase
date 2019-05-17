package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.buttons.Button;
import org.team5940.pantry.experimental.command.SendableCommandBase;

public class WaitForOI extends SendableCommandBase {
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
	public void initialize() {
		this.clearRequirements();
		toRun.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return toPoll.get();
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
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
