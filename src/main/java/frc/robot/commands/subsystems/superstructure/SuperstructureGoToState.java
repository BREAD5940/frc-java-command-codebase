package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.SuperStructureState;

public class SuperstructureGoToState extends Command {
	SuperStructureState mRequState;

	public SuperstructureGoToState(SuperStructureState requState) {
		// requires(SuperStructure.getInstance());
		mRequState = requState;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// SuperStructure.getInstance().setReqState(mRequState);
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
