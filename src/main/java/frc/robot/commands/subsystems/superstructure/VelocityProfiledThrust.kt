package frc.robot.commands.subsystems.superstructure

import edu.wpi.first.wpilibj.command.Command
import frc.robot.lib.obj.RoundRotation2d
import frc.robot.states.SuperStructureState
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.PI
import kotlin.math.cos

@Suppress("unused")
class VelocityProfiledThrust(private val goalProximal: RoundRotation2d) : Command() {

    val structure by lazy { SuperStructure.getInstance() }
    lateinit var initialState: SuperStructureState
    private lateinit var endSetpoint: SuperStructureState

    override fun initialize() {
//        initialState // TODO this should run the lazy initilizer
        val currentState = SuperStructure.getInstance().currentState
        initialState = currentState

        val goalWrist = SuperStructure.getDumbWrist(0.roundDegree, goalProximal)

        val goalProximalVector = Translation2d(ProximalThrust.kProximalLen, goalProximal.toRotation2d())
        val goalElevator = currentState.elevatorHeight - goalProximalVector.y

        endSetpoint = SuperStructureState(
                goalElevator, goalProximal, goalWrist
        )

        structure.move(endSetpoint)
    }

    override fun execute() {
        val state = structure.currentState

        SuperStructure.elevator.master.motionCruiseVelocity = calculateElevatorSpeed(state.elbowAngle.toRotation2d())
        structure.elbow.master.setMotionCruiseVelocity(
                kProximalSpeed.velocity
        )
        structure.wrist.master.setMotionCruiseVelocity(
                kWristSpeed.velocity
        )

        structure.move(endSetpoint)
    }

    override fun isFinished() =  structure.isWithinToleranceOf(endSetpoint)

    fun calculateElevatorSpeed(proximalAngle: Rotation2d): LinearVelocity {

        // so when the proximal angle is -90 or +90, this needs to return 0
        // when the proximal is at 0 this needs to return the linear velocity of the tip of the proximal given kproximal speed
        // it should be a trig relationship between the two

        val proximalCircumference = /* diameter */ (kElbowLength * 2) * PI
        val proximalTipVelocity = (proximalCircumference * (kProximalSpeed.degree / 360)).velocity
        return proximalTipVelocity * proximalAngle.cos
    }

    companion object {

        val kProximalSpeed = 30.degree // per second
        val kWristSpeed = kProximalSpeed / 2
        val kElbowLength = 0.41.meter


    }

}