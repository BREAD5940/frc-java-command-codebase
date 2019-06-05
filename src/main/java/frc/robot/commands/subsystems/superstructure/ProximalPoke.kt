package frc.robot.commands.subsystems.superstructure

import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.robot.lib.obj.RoundRotation2d
import frc.robot.states.ElevatorState
import frc.robot.states.SuperStructureState
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import frc.robot.subsystems.superstructure.SuperStructure.getDumbWrist
//import frc.robot.subsystems.superstructure.SuperStructure.getInstance
import org.ghrobotics.lib.mathematics.units.*
import org.team5940.pantry.exparimental.command.SendableCommandBase

val Number.roundRadian: RoundRotation2d get() = RoundRotation2d.getRadian(toDouble())
@Suppress("unused")
val Number.roundDegree: RoundRotation2d get() = RoundRotation2d.getDegree(toDouble())

class ProximalPoke(
        private val structure: SuperStructure,
        private val elevator : Elevator,
        private val pokeDistance: Length,
        private val wristAngle: RoundRotation2d,
        private val isOutwardsPoke: Boolean = true
) : Command() {

    companion object {

        fun OutwardPokeTo(distance:Length) = ProximalPoke(
                SuperStructure.getInstance(),
                SuperStructure.elevator,
                distance,
                0.roundDegree,
                true
        )

        fun RetractTo(distance:Length) = ProximalPoke(
                SuperStructure.getInstance(),
                SuperStructure.elevator,
                distance,
                0.roundDegree,
                false
        )

        val kProximalLen = 17.inch.meter
        val kProximalSpeed = Math.toRadians(50.0)

    }

    init {
        requires(structure)
        requires(structure.elbow)
        requires(structure.wrist)
        requires(elevator)
    }

    var startPose: Translation2d? = null
    var startState: SuperStructureState? = null
    var goalPose: Translation2d? = null
    var lastState = SuperStructureState()
    var lastProximalAngle = 0.0
    var iteratorFinished = false

    override fun initialize() {
        startState = structure.currentState
        startPose = SuperStructure.getCurrentStateAsPose(startState)
        goalPose = startPose!! + Translation2d(pokeDistance, Length.kZero)
        lastState = startState!!
    }

    override fun execute() {
        val currentState = structure.currentState

        // haha i pulled a sneeky on ya. Double times Boolean is Double. if true, the Boolean is 1, or -1 if false
        val newWantedProximal = lastState.elbowAngle.radian + (kProximalSpeed * 0.02 * isOutwardsPoke)

        val startProximalTranslation2d = Translation2d(kProximalLen.meter, startState!!.elbowAngle.toRotation2d()) // Translation2d.fromRotation2d(startState!!.elbowAngle.radian, kProximalLen)

        val newProximalTranslation2d = Translation2d(kProximalLen.meter, newWantedProximal.radian) //Translation2d.fromRotation2d(newWantedProximal, kProximalLen)

        if (newProximalTranslation2d.x > pokeDistance && isOutwardsPoke
                || newProximalTranslation2d.x < pokeDistance && !isOutwardsPoke) {
            iteratorFinished = true ; return
        }

        val deltaHeight = newProximalTranslation2d.y - startProximalTranslation2d.y

        val newWantedElevator = startState!!.elevatorHeight + deltaHeight

        val newWantedWrsit = getDumbWrist(wristAngle, currentState.elbowAngle)

        structure.elbow.requestAngle(newWantedProximal.roundRadian)
        structure.wrist.requestAngle(newWantedWrsit)
        elevator.setPositionSetpoint(ElevatorState(newWantedElevator))

        this.lastState = currentState

    }

    override fun isFinished(): Boolean {
        return iteratorFinished
    }

    override fun initSendable(builder: SendableBuilder?) {

        builder?.addBooleanProperty("iterator finished", { iteratorFinished }, null)
        builder?.addStringProperty("last commanded state", { lastState.toString() }, null)

        super.initSendable(builder)
    }

}

private operator fun Number.times(other: Boolean): Double {
    return toDouble() * (if (other) 1 else -1)
}