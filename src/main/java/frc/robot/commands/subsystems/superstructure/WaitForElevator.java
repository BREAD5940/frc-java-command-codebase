package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.experimental.command.SendableCommandBase;

public class WaitForElevator extends SendableCommandBase {

	double demand;

	public WaitForElevator(double demand) {
		// Doesn't require anything, just waiting for elevator height
		this.demand = demand;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		// return Robot.elevator.isWithinTolerence(demand);
		return true;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
