package frc.robot.commands.subsystems.superstructure

import edu.wpi.first.wpilibj.command.Command
import frc.robot.lib.obj.RoundRotation2d
import frc.robot.states.ElevatorState
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*

val Number.roundRadian: RoundRotation2d get() = RoundRotation2d.getRadian(toDouble())
val Number.roundDegree: RoundRotation2d get() = RoundRotation2d.getDegree(toDouble())

class ProximalThrust : Command() {

    val structure
        get() = SuperStructure.getInstance()

    var outwardPokeDone = false
    var readyForExit = false
    var initialHeight: Length? = null
    var initialAngle: Rotation2d? = null
    var lastAngle = initialAngle

    companion object {
        val kProximalLen = 17.inch
        val kProximalSpeed = 80.degree / 1.second
        val resetAngle = (-58).degree
    }

    init {
        isInterruptible = false
    }

    override fun initialize() {

        val proximalSpeedTicks = structure.elbow.master.getTicks(kProximalSpeed * 1.second)
        structure.elbow.master.configMotionCruiseVelocity(proximalSpeedTicks)

        val elbowSpeedTicks = structure.wrist.master.getTicks(kProximalSpeed / 2 * 1.second * 1.1)
        structure.wrist.master.configMotionCruiseVelocity(elbowSpeedTicks)

        outwardPokeDone = false
        readyForExit = false

        val currentState = structure.currentState

        val proximalVector = Translation2d(kProximalLen, currentState.elbowAngle.toRotation2d())
        val elevatorVector = Translation2d(Length.kZero, currentState.elevatorHeight)

        initialHeight = (proximalVector + elevatorVector).y
        initialAngle = currentState.elbowAngle.toRotation2d()
        lastAngle = initialAngle
    }

    override fun execute() {

        if (readyForExit) return

        // we be poking bois
        val currentState = structure.currentState
        val nextProximalAngle = if (!outwardPokeDone) {
            lastAngle!! + kProximalSpeed * 20.millisecond
        } else {
            lastAngle!! - kProximalSpeed * 20.millisecond
        }

        if (nextProximalAngle.degree > 0 && !outwardPokeDone) { outwardPokeDone = true ; return }
        if (nextProximalAngle < resetAngle) {
            outwardPokeDone = true ; readyForExit = true ; return
        }

        val nextWrist = SuperStructure.getDumbWrist(0.roundDegree, currentState.elbowAngle)

        val proximalVector = Translation2d(kProximalLen, currentState.elbowAngle.toRotation2d())
        val elevatorNeeded = initialHeight!! - proximalVector.y

        structure.elbow.requestAngle(nextProximalAngle.toRoundRotation())
        structure.wrist.requestAngle(nextWrist)
        SuperStructure.elevator.setPositionSetpoint(ElevatorState(elevatorNeeded))

        lastAngle = nextProximalAngle
    }

    override fun end() = interrupted()

    override fun interrupted() {
        structure.elbow.master.configMotionCruiseVelocity(((1250 / 0.7).toInt()))
        structure.wrist.master.configMotionCruiseVelocity(((2000).toInt()))
    }

    override fun isFinished() = outwardPokeDone && readyForExit
}

private fun Rotation2d.toRoundRotation() = RoundRotation2d.fromRotation2d(this@toRoundRotation)
