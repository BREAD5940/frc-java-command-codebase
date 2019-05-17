package frc.robot.commands.subsystems.superstructure;

import java.util.concurrent.Callable;

import org.ghrobotics.lib.mathematics.units.Length;

import org.team5940.pantry.experimental.command.SendableCommandBase;
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

	/**
	 * Move the arm after waiting for the elevator to attain a desired state. This assumes that the elevator setpoint has already been set!
	 * @param desired the desired intake angles after the elevator moves
	 * @param finalEleHeight the elevator height to wait for
	 * @param tolerence the tolerance about the elevator setpoint
	 * @param isDescending if the elevator is going down or not
	 */
	public ArmWaitForElevator(IntakeAngle desired, Length finalEleHeight, Length tolerence) {
		elevatorMoved = (() -> {
			// return ((SuperStructure.getElevator().getHeight().getInch() < finalEleHeight.plus(tolerence).getInch() && isDescending)
			// 		|| (SuperStructure.getElevator().getHeight().getInch() > finalEleHeight.minus(tolerence).getInch()
			// 				&& !isDescending));
			return (SuperStructure.getElevator().getHeight().minus(finalEleHeight).getAbsoluteValue().getInch() < tolerence.getInch()); // check if the position error is acceptable
		}); // this lamda Caller can be read as:
		// (null) -> [or becomes, turns into] (basically if the elevator is within tolerance)
		// this Caller is then used by auto wait for condition and polled in isFinished();

		addSequential(new AutoWaitForCondition(elevatorMoved));
		addSequential(new ArmMove(desired));
	}

	@Override
	public boolean isFinished() {
		return Math.abs(desired.wristAngle.angle.getDegree() - SuperStructure.getInstance().getWrist().getDegrees()) <= 2
				|| Math.abs(desired.elbowAngle.angle.getDegree() - SuperStructure.getInstance().getElbow().getDegrees()) <= 2;
	}

}
