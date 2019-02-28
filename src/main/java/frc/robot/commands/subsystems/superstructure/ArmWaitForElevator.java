package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.IntakeAngle;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ArmWaitForElevator extends Command {

	IntakeAngle desired;
	Length finalEleHeight, tolerence;
	boolean isDescending;

	public ArmWaitForElevator(IntakeAngle desired, Length finalEleHeight, Length tolerence, boolean isDescending) {
		this.desired = desired;
		this.finalEleHeight = finalEleHeight;
		this.tolerence = tolerence;
		this.isDescending = isDescending;
	}

	@Override
	protected void initialize() {
		clearRequirements(); //make sure we aren't bothering anyone
	}

	@Override
	protected void execute() {
		if ((SuperStructure.getElevator().getHeight().getInch() < finalEleHeight.plus(tolerence).getInch() && isDescending)
				|| (SuperStructure.getElevator().getHeight().getInch() > finalEleHeight.minus(tolerence).getInch()
						&& !isDescending)) {
			requires(SuperStructure.getInstance().getWrist());
			requires(SuperStructure.getInstance().getElbow());
			SuperStructure.getInstance().getWrist().requestAngle(desired.getWrist().angle);
			SuperStructure.getInstance().getElbow().requestAngle(desired.getElbow().angle);
		}
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(desired.wristAngle.angle.getDegree() - SuperStructure.getInstance().getWrist().getDegrees()) <= 2
				|| Math.abs(desired.elbowAngle.angle.getDegree() - SuperStructure.getInstance().getElbow().getDegrees()) <= 2;
	}

}
