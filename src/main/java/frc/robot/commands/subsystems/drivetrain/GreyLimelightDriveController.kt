/* Some algorithms and implementation from Team 973 Greybots */

package frc.robot.commands.subsystems.drivetrain

import com.team1323.lib.util.SynchronousPIDF
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.ghrobotics.vision.TargetTracker
import frc.robot.Robot
import frc.robot.lib.motion.Util
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.LimeLight
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.opencv.core.Point
import java.lang.Math.abs
import kotlin.math.absoluteValue
import kotlin.math.withSign

@Suppress("unused")
class GreyLimelightDriveController(
    val skewCompEnabled: Boolean,
    val wantsHatchMode: Boolean
) : Command() {

    private var isOnTarget = false

    private val forwardPID = SynchronousPIDF(
            0.15, 0.0, 0.00
    )
    private val turnPID = SynchronousPIDF(
            0.8, 0.0, 8.0
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

//        DriveTrain.getInstance().setRobotPosition(TrajectoryWaypoints.kCargoShipFL)
    }

    override fun execute() {

        // check for valid targets and return otherwise
        if (!limeLight.hasTarget()) {
            m_rightSetpoint = 0.0
            m_leftSetpoint = 0.0
            return
        }

        val newTarget = TargetTracker.getBestTargetUsingReference(DriveTrain.getInstance().robotPosition,
                !SuperStructure.getInstance().isPassedThrough) ?: return

        if (!newTarget.isAlive) return

        val transform = newTarget.averagedPose2d inFrameOfReferenceOf DriveTrain.getInstance().robotPosition
        var angleError = -(transform.translation.toRotation()) // limeLight.txDegrees

        while (angleError > 180.degree) angleError -= 180.degree // constrain to [-180, 180]
        while (angleError < -180.degree) angleError += 180.degree

        val distance = transform.translation.norm
        val distanceError = distance - if (wantsHatchMode) DISTANCE_SETPOINT_HATCH else DISTANCE_SETPOINT_CARGO

        val throttlePIDOut = Robot.m_oi.forwardAxis
                // forwardPID.calculate(-distanceError.inch, 0.020)

        val turnPIDOut = turnPID.calculate((angleError -
                if (wantsHatchMode) HATCH_VISION_OFFSET else CARGO_VISION_OFFSET).radian, 0.020)

        val goalAngleCompensation = calcScaleGoalAngleComp(distance, angleError)

        if (skewCompEnabled) {
            m_leftSetpoint = throttlePIDOut - turnPIDOut - goalAngleCompensation
            m_rightSetpoint = throttlePIDOut + turnPIDOut + goalAngleCompensation
        } else {
            m_leftSetpoint = throttlePIDOut - turnPIDOut
            m_rightSetpoint = throttlePIDOut + turnPIDOut
        }

        // sticktion compensation voltage maybe
//        m_leftSetpoint += (1.0 / 12.0).withSign(m_leftSetpoint)
//        m_rightSetpoint += (1.0 / 12.0).withSign(m_rightSetpoint)

        println("GreyLimelight distanceError{${distanceError.inch}} angleError {$angleError} throttle{$throttlePIDOut} turn{$turnPIDOut}" +
                "${if (skewCompEnabled) " skewComp{$goalAngleCompensation" else ""} left{$m_leftSetpoint} right{$m_rightSetpoint} ")

        DriveTrain.getInstance().setPowers(m_leftSetpoint, m_rightSetpoint)

        isOnTarget = angleError.absoluteValue < 5.0.degree && distanceError.inch.absoluteValue < 3.0 && throttlePIDOut < 0.3 && DriveTrain.getInstance().gyro.velocityZ.absoluteValue < 3.0
    }

    private fun calcScaleGoalAngleComp(distance: Length, xOffsetDegrees: Rotation2d): Double {

        val distMultiplier = (
                Util.interpolate(
                        Point(GOAL_ANGLE_COMP_DISTANCE_MIN, 0.0),
                        Point(GOAL_ANGLE_COMP_DISTANCE_MAX, 1.0),
                        distance.inch)
                ).boundTo(0.0, 1.0)

        val skew = limeLight.targetSkew
        val frame_multiplier = (Util.interpolate(
                Point(SKEW_COMP_MULTIPLIER_DISTANCE_MIN, 1.0),
                Point(SKEW_COMP_MULTIPLIER_DISTANCE_MAX, 0.0),
                abs(xOffsetDegrees.degree)
        )).boundTo(0.0, 1.0)

        val skewComp = (GOAL_ANGLE_COMP_KP * skew * frame_multiplier * distMultiplier).boundTo(
                SKEW_MIN, SKEW_MAX)

        return -skewComp; // y = mx + b
                            // y = degree of compensation
                            // m = (1 - 0) / (max - min)
                            // x = distance to target
                            // b = y-int as plugged in to slope intercept equation
    }

    companion object {

        val DISTANCE_SETPOINT_CARGO = 30.inch
        val DISTANCE_SETPOINT_HATCH = 30.inch
        const val GOAL_ANGLE_COMP_DISTANCE_MIN = 24.0
        const val GOAL_ANGLE_COMP_DISTANCE_MAX = 60.0
        const val SKEW_COMP_MULTIPLIER_DISTANCE_MIN = 17.0
        const val SKEW_COMP_MULTIPLIER_DISTANCE_MAX = 24.0
        const val GOAL_ANGLE_COMP_KP = 0.008
        const val SKEW_MIN = -0.2
        const val SKEW_MAX = 0.2

        val HATCH_VISION_OFFSET = 0.0.degree
        val CARGO_VISION_OFFSET = 0.0.degree

        const val MAX_TURN_MAGNITUDE = 0.4
        const val MAX_FORWARD_MAGNITUDE = 0.6 / 1.5

        private var m_rightSetpoint = 0.0
        private var m_leftSetpoint = 0.0
    }

    override fun isFinished() = isOnTarget

    override fun initSendable(builder: SendableBuilder) {

        builder.addDoubleProperty("forwardKp", { forwardPID.p }, { newP -> forwardPID.setPID(newP, forwardPID.i, forwardPID.d) })
        builder.addDoubleProperty("turnKp", { turnPID.p }, { newP -> turnPID.setPID(newP, turnPID.i, turnPID.d) })

        super.initSendable(builder)
    }
}

private fun Translation2d.toRotation() = Rotation2d(x.value, y.value, true)

private fun Number.boundTo(low: Double, high: Double): Double {
    return when {
        toDouble() < low -> low
        toDouble() > high -> high
        else -> toDouble()
    }
}
