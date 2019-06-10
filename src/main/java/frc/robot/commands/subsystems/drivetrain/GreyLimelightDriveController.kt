/* Some algorithms and implementation from Team 973 Greybots */

package frc.robot.commands.subsystems.drivetrain

import com.team1323.lib.util.SynchronousPIDF
import edu.wpi.first.wpilibj.command.Command
import frc.robot.lib.Logger
import frc.robot.lib.motion.Util
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.LimeLight
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.inch
import org.opencv.core.Point
import java.lang.Math.abs
import kotlin.math.absoluteValue

@Suppress("unused")
class GreyLimelightDriveController(
    val skewCompEnabled: Boolean,
    val wantsHatchMode: Boolean
) : Command() {

    private var isOnTarget = false

    private val forwardPID = SynchronousPIDF(
            0.01, 0.0, 0.001
    )
    private val turnPID = SynchronousPIDF(
            0.01, 0.0, 0.001
    )

    init {
        forwardPID.setContinuous(false)
        forwardPID.setOutputRange(-MAX_FORWARD_MAGNITUDE, MAX_FORWARD_MAGNITUDE)

        turnPID.setContinuous(false)
        turnPID.setOutputRange(-MAX_TURN_MAGNITUDE, MAX_TURN_MAGNITUDE)
    }

    private val limeLight by lazy { LimeLight.getInstance() }

    override fun initialize() {
        isOnTarget = false
        m_rightSetpoint = 0.0
        m_leftSetpoint = 0.0
        limeLight.turnOnLED()
    }

    override fun execute() {

        // check for valid targets and return otherwise
        if (!limeLight.hasTarget()) {
            m_rightSetpoint = 0.0
            m_leftSetpoint = 0.0
            return
        }

        val angleError = limeLight.dx.degree

        val distance = limeLight.estimateDistanceFromAngle()
        val distanceError = distance - if (wantsHatchMode) DISTANCE_SETPOINT_HATCH else DISTANCE_SETPOINT_CARGO

        val throttlePIDOut = forwardPID.calculate(
                -distanceError.inch, 0.020
        )

        val turnPIDOut = turnPID.calculate(angleError -
                if (wantsHatchMode) HATCH_VISION_OFFSET else CARGO_VISION_OFFSET, 0.020)

        val goalAngleCompensation = calcScaleGoalAngleComp(distance, angleError)

        if (skewCompEnabled) {
            m_leftSetpoint = throttlePIDOut - turnPIDOut - goalAngleCompensation
            m_rightSetpoint = throttlePIDOut + turnPIDOut + goalAngleCompensation
        } else {
            m_leftSetpoint = throttlePIDOut - turnPIDOut
            m_rightSetpoint = throttlePIDOut + turnPIDOut
        }

        Logger.log("GreyLimelight throttle{$throttlePIDOut} turn{$turnPIDOut}" +
                "${if (skewCompEnabled) " skewComp{$goalAngleCompensation" else ""} left{$m_leftSetpoint} right{$m_rightSetpoint} ")

        DriveTrain.getInstance().setPowers(m_leftSetpoint, m_rightSetpoint)

        isOnTarget = abs(angleError) < 5.0 && distanceError.inch.absoluteValue < 3.0 && throttlePIDOut < 0.3 && DriveTrain.getInstance().gyro.velocityZ.absoluteValue < 3.0
    }

    private fun calcScaleGoalAngleComp(distance: Length, xOffsetDegrees: Double): Double {

        val distMultiplier = (
                Util.interpolate(
                        Point(GOAL_ANGLE_COMP_DISTANCE_MIN, 0.0),
                        Point(GOAL_ANGLE_COMP_DISTANCE_MAX, 1.0),
                        distance.meter)
                ).boundTo(0.0, 1.0)

        val skew = limeLight.targetSkew
        val frame_multiplier = Util.interpolate(
                Point(SKEW_COMP_MULTIPLIER_DISTANCE_MIN, 1.0),
                Point(SKEW_COMP_MULTIPLIER_DISTANCE_MAX, 0.0),
                abs(xOffsetDegrees)
        ).boundTo(0.0, 1.0)

        val skewComp = (GOAL_ANGLE_COMP_KP * skew * frame_multiplier * distMultiplier).boundTo(
                SKEW_MIN, SKEW_MAX)

        return -skewComp; // y = mx + b
                            // y = degree of compensation
                            // m = (1 - 0) / (max - min)
                            // x = distance to target
                            // b = y-int as plugged in to slope intercept equation
    }

    companion object {

        val DISTANCE_SETPOINT_CARGO = 20.inch
        val DISTANCE_SETPOINT_HATCH = 20.inch
        const val GOAL_ANGLE_COMP_DISTANCE_MIN = 24.0
        const val GOAL_ANGLE_COMP_DISTANCE_MAX = 60.0
        const val SKEW_COMP_MULTIPLIER_DISTANCE_MIN = 17.0
        const val SKEW_COMP_MULTIPLIER_DISTANCE_MAX = 24.0
        const val GOAL_ANGLE_COMP_KP = 0.008
        const val SKEW_MIN = -0.2
        const val SKEW_MAX = 0.2

        const val HATCH_VISION_OFFSET = 0.0
        const val CARGO_VISION_OFFSET = 0.0

        const val MAX_TURN_MAGNITUDE = 0.4
        const val MAX_FORWARD_MAGNITUDE = 0.6

        private var m_rightSetpoint = 0.0
        private var m_leftSetpoint = 0.0
    }

    override fun isFinished() = isOnTarget
}

private fun Number.boundTo(low: Double, high: Double): Double {
    return when {
        toDouble() < low -> low
        toDouble() > high -> high
        else -> toDouble()
    }
}
