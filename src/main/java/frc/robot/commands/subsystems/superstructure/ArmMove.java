package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.exparimental.command.SendableCommandBase;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.IntakeAngle;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ArmMove extends SendableCommandBase {

	IntakeAngle mDesired;

	/**
	 * Move the intake joints to a desired state and wait for them to get there
	 * @param state the desired IntakeAngle state (assumed to be safe!)
	 */
	public ArmMove(IntakeAngle state) {
		this.mDesired = state;
		addRequirements(SuperStructure.getInstance().getWrist());
		addRequirements(SuperStructure.getInstance().getElbow());
	}

	@Override
	public void execute() {
		SuperStructure.getInstance().getWrist().requestAngle(mDesired.getWrist().angle);
		SuperStructure.getInstance().getElbow().requestAngle(mDesired.getElbow().angle);
	}

	@Override
	public boolean isFinished() {
		return SuperStructure.getInstance().getWrist().isWithinTolerance(RoundRotation2d.getDegree(5), mDesired.getWrist().angle)
				&& SuperStructure.getInstance().getElbow().isWithinTolerance(RoundRotation2d.getDegree(5), mDesired.getElbow().angle);
	}

}
