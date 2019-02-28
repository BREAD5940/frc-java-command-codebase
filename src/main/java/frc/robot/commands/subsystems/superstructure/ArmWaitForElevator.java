package frc.robot.commands.subsystems.superstructure;

import java.util.concurrent.Callable;

import org.ghrobotics.lib.mathematics.units.Length;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.IntakeAngle;
import frc.robot.subsystems.superstructure.SuperStructure;

import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.lib.AutoWaitForCondition;
import frc.robot.states.IntakeAngle;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ArmWaitForElevator extends AutoCommandGroup {

	IntakeAngle desired;
	Length finalEleHeight, tolerence;
	boolean isDescending;
	Callable<Boolean> elevatorMoved;
	Command mDelayedCommand;

	public ArmWaitForElevator(IntakeAngle desired, Length finalEleHeight, Length tolerence, boolean isDescending) {
		elevatorMoved = (() -> {
			return ((SuperStructure.getElevator().getHeight().getInch() < finalEleHeight.plus(tolerence).getInch() && isDescending)
					|| (SuperStructure.getElevator().getHeight().getInch() > finalEleHeight.minus(tolerence).getInch()
							&& !isDescending));
		}); // this lamda Caller can be read as:
		// (null) -> [or becomes, turns into] (basically if the elevator is within tolerance)
		// this Caller is then used by auto wait for condition and polled in isFinished();

		addSequential(new AutoWaitForCondition(elevatorMoved));
		addSequential(new ArmMove(desired));
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(desired.wristAngle.angle.getDegree() - SuperStructure.getInstance().getWrist().getDegrees()) <= 2
				|| Math.abs(desired.elbowAngle.angle.getDegree() - SuperStructure.getInstance().getElbow().getDegrees()) <= 2;
	}

}
