package frc.robot.commands.subsystems.drivetrain

import edu.wpi.first.wpilibj.command.Command
import frc.robot.Robot
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.LimeLight
import frc.robot.subsystems.superstructure.SuperStructure
//import org.apache.commons.math3.ml.neuralnet.Network
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.radian
import org.ghrobotics.lib.utils.DoubleSource
import kotlin.math.abs
import kotlin.math.absoluteValue

//import org.team5940.pantry.exparimental.command.Command

class DualHybridDriverAssist : DriveTrain.CurvatureDrive() {

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override fun initialize() {
        referencePose = DriveTrain.getInstance().robotPosition
    }

    override fun execute() {

        var isFront = SuperStructure.getInstance().isPassedThrough

        var turnInput: Double?

        if(isFront) {
            // it's LimeLight Time
            val lemonLight = LimeLight.getInstance()
            val hasTarget = lemonLight.trackedTargets > 0.5

            // check that we have a target
            if (!hasTarget) {
                turnInput = null
                println("no vision targets found")
            } else {
                println("limelight found vision target")
                val dx = lemonLight.dx.radian
                turnInput = dx
            }
        } else {
            // is YeeVois Time
            val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, false)

            val newPose = newTarget?.averagedPose2d
            if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

            val lastKnownTargetPose = this.lastKnownTargetPose
            if(lastKnownTargetPose == null) {
                turnInput = null
                println("no vision targets found")
            } else {
                println("jevois target found")
                val transform = lastKnownTargetPose inFrameOfReferenceOf DriveTrain.getInstance().robotPosition // TODO check math
                val angle = Rotation2d(transform.translation.x.value, transform.translation.y.value, true)

//                Network.visionDriveAngle.setDouble(angle.degree)
//                Network.visionDriveActive.setBoolean(true)

                val angleError = angle + if (isFront) Rotation2d.kZero else Math.PI.radian

                if(angleError.degree.absoluteValue > 45) {
                    // plz no disable us when going to loading station, kthx
                    this.lastKnownTargetPose = null
                    turnInput = null
                } else {
                    turnInput = angleError.radian
                }
            }

        }

        // check if our vision even sees anything - if not, normal drive time
        if(turnInput == null) {
            super.execute()
        } else {

            var forward = Robot.m_oi.forwardAxis
            forward *= forward
            if (DriveTrain.getInstance().cachedGear == DriveTrain.Gear.HIGH) {
                forward *= 0.8
            }

            val turn = kCorrectionKp * turnInput + kCorrectionKd * (turnInput - prevError)

            DriveTrain.getInstance().tankDrive(forward, turn)

            prevError = turnInput
        }

    }

    companion object {
        const val kCorrectionKp = 0.8
        const val kCorrectionKd = 8.0
    }

    override fun isFinished() = false


}

