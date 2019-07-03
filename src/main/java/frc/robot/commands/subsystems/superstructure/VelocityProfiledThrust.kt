package frc.robot.commands.subsystems.superstructure

import edu.wpi.first.wpilibj.command.Command
import frc.robot.commands.auto.routines.AutoRoutine
import frc.robot.commands.subsystems.superstructure.ProximalThrust.Companion.kProximalLen
import frc.robot.lib.obj.RoundRotation2d
import frc.robot.states.SuperStructureState
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.SuperStructure
import org.apache.commons.math3.geometry.euclidean.threed.Rotation
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.toTranslation
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import kotlin.math.PI
import kotlin.math.cos

//@Suppress("unused")
class VelocityProfiledThrust(private val goalProximal: RoundRotation2d) : Command() {

    val structure by lazy { SuperStructure.getInstance() }
    lateinit var initialState: SuperStructureState
    private lateinit var endSetpoint: SuperStructureState

    override fun initialize() {
        val currentState = SuperStructure.getInstance().currentState
        initialState = currentState

        val goalWrist = SuperStructure.getDumbWrist(0.roundDegree, goalProximal)


        println("the proximal will be ${(goalProximal.toTranslation() * kProximalLen.meter).y.inch} down")
        println("the proximal is currently " +
                "${(currentState.elbowAngle.toRotation2d().toTranslation() * kProximalLen.meter).y.inch} down!")

        val deltaElevator = /* end minus current */ (currentState.elbowAngle.toRotation2d().toTranslation() * kProximalLen.meter) -
                (goalProximal.toTranslation() * kProximalLen.meter)

        val endElevator = currentState.elevatorHeight + deltaElevator.y

        endSetpoint = SuperStructureState(
                endElevator, goalProximal, goalWrist
        )

        println("This move will move the elevator by ${deltaElevator.y.inch} inches!")

        println("because we started at ${currentState.elevatorHeight.inch} inches, we will" +
                "target a height of ${endElevator.inch} inches!")

        structure.move(endSetpoint)
    }

    override fun execute() {
        val state = structure.currentState

        SuperStructure.elevator.master.configMotionCruiseVelocity(
                SuperStructure.elevator.lengthModel.toNativeUnitVelocity(
                        (calculateElevatorSpeed(state.elbowAngle.toRotation2d())).value / 10.0
                ).toInt()
        )

        structure.elbow.master.setMotionCruiseVelocity(
                kProximalSpeed
        )
        structure.wrist.master.setMotionCruiseVelocity(
                kWristSpeed
        )

        structure.move(endSetpoint)

    }

    override fun interrupted() {
        SuperStructure.getInstance().move(structure.currentState)
        end()
    }

    override fun end() {
        SuperStructure.elevator.configMotionMagicGains(Elevator.HIGH_GEAR_MOTION_MAGIC)
        structure.wrist.setMotionMagicGains()
        structure.elbow.setMotionMagicGains()
    }

    override fun isFinished() = structure.isWithinToleranceOf(endSetpoint)

    fun calculateElevatorSpeed(proximalAngle: Rotation2d): LinearVelocity {

        // so when the proximal angle is -90 or +90, this needs to return 0
        // when the proximal is at 0 this needs to return the linear velocity of the tip of the proximal given kproximal speed
        // it should be a trig relationship between the two

        println("proximalLinearVelocity ${proximalLinearVelocity.value.meter.inch} inches per sec")

        val cosine = (proximalAngle.boundTo((-90).degree, 90.degree)).cos

        println("cos at this angle is $cosine")

        val toReturn: LinearVelocity = (proximalLinearVelocity * cosine).absoluteValue

        println("velocity at angle ${proximalAngle.degree} is ${toReturn.value.meter.inch}")

        return toReturn.absoluteValue
    }

    companion object {

        val kProximalSpeed = 50.degree.velocity // per second
        val kWristSpeed = kProximalSpeed / 2
        private val kElbowLength = 0.41.meter

        private val proximalCircumference = /* diameter */ (kElbowLength * 2) * PI
        val proximalLinearVelocity = (proximalCircumference.meter*((kProximalSpeed * 1.second).degree / 360.0)).meter.velocity




        // 0.35 meters per second

        fun getTestCommand() = object : AutoRoutine() {

            init {
//                +JankyGoToState(SuperStructure.iPosition.HATCH_GRAB_INSIDE)
                +VelocityProfiledThrust(0.roundDegree)
//                +WaitCommand(0.5)
//                +VelocityProfiledThrust((-45).roundDegree)
            }
        }
    }
}

private fun Rotation2d.boundTo(min: Rotation2d, max: Rotation2d) = when {
    this > max -> max
    this < min -> min
    else -> this
}

private fun RoundRotation2d.toTranslation(): Translation2d {
    return toRotation2d().toTranslation()
}
