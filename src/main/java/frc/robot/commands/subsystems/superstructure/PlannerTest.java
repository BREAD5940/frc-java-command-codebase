
package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.planners.SuperstructureMotion;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class PlannerTest extends SendableCommandBase {
	SuperStructureState goal;

	public PlannerTest(SuperStructureState goal) {
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		SuperstructureMotion.getInstance().plan(goal, SuperStructure.getInstance().lastState);
		// SuperstructureMotion.getInstance().schedule();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return true; // are you sure about that....
		// if this command exits will superstructureteleop do an oof?
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	// @Override
	// protected void interrupted() {}
}
