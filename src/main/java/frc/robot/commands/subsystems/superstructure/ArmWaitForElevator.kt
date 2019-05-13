package frc.robot.commands.subsystems.superstructure


import org.ghrobotics.lib.mathematics.units.Length

import frc.robot.lib.AutoWaitForCondition
import frc.robot.states.IntakeAngle
import frc.robot.subsystems.superstructure.SuperStructure
import org.team5940.pantry.exparimental.command.SequentialCommandGroup

class ArmWaitForElevator
/**
 * Move the arm after waiting for the elevator to attain a desired state. This assumes that the elevator setpoint has already been set!
 * @param desired the desired intake angles after the elevator moves
 * @param finalEleHeight the elevator height to wait for
 * @param tolerence the tolerance about the elevator setpoint
 */
(desired: IntakeAngle, finalEleHeight: Length, tolerence: Length) : SequentialCommandGroup() {

    internal var desired: IntakeAngle? = null
    internal var elevatorMoved: () -> Boolean // dont join these, otherwise it will do a big oof

    init {
        elevatorMoved = {
            // return ((SuperStructure.getElevator().getHeight().getInch() < finalEleHeight.plus(tolerence).getInch() && isDescending)
            // 		|| (SuperStructure.getElevator().getHeight().getInch() > finalEleHeight.minus(tolerence).getInch()
            // 				&& !isDescending));
            SuperStructure.getElevator().height.minus(finalEleHeight).absoluteValue.inch < tolerence.inch // check if the position error is acceptable
        } // this lamda Caller can be read as:
        // (null) -> [or becomes, turns into] (basically if the elevator is within tolerance)
        // this Caller is then used by auto wait for condition and polled in isFinished();

        addCommands(AutoWaitForCondition(elevatorMoved), ArmMove(desired))
    }

    override fun isFinished(): Boolean {
        return Math.abs(desired!!.wristAngle.angle.degree - SuperStructure.getInstance().wrist.degrees) <= 2 || Math.abs(desired!!.elbowAngle.angle.degree - SuperStructure.getInstance().elbow.degrees) <= 2
    }

}
