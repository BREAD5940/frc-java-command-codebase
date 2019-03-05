package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.IntakeAngle;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ArmMove extends Command {

	IntakeAngle mDesired;

	/**
	 * Move the intake joints to a desired state and wait for them to get there
	 * @param state the desired IntakeAngle state (assumed to be safe!)
	 */
	public ArmMove(IntakeAngle state) {
		this.mDesired = state;
		requires(SuperStructure.getInstance().getWrist());
		requires(SuperStructure.getInstance().getElbow());
	}

	@Override
	protected void execute() {
		SuperStructure.getInstance().getWrist().requestAngle(mDesired.getWrist().angle);
		SuperStructure.getInstance().getElbow().requestAngle(mDesired.getElbow().angle);
	}

	@Override
	protected boolean isFinished() {
		return SuperStructure.getInstance().getWrist().isWithinTolerance(RoundRotation2d.getDegree(5), mDesired.getWrist().angle)
				&& SuperStructure.getInstance().getElbow().isWithinTolerance(RoundRotation2d.getDegree(5), mDesired.getElbow().angle);
	}

}
