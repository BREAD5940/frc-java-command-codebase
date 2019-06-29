// implementation from Team 5190 Green Hope Robotics

package frc.robot.commands.subsystems.drivetrain

import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.ghrobotics.vision.TargetTracker
import frc.robot.Network
import frc.robot.Robot
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.radian
import kotlin.math.absoluteValue

class VisionDriveCommand : Command() {

    override fun isFinished() = false

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override fun initialize() {
        isActive = true
        referencePose = DriveTrain.getInstance().robotPosition
    }

    override fun execute() {

        val isFront = !SuperStructure.getInstance().isPassedThrough

        println("IS FRONT? $isFront")

        val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, isFront)

        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

        val source = Robot.m_oi.forwardAxis

        if (lastKnownTargetPose == null) {
//            ElevatorSubsystem.wantedVisionMode = true
            super.execute()
        } else {
//            ElevatorSubsystem.wantedVisionMode = false
            val transform = lastKnownTargetPose inFrameOfReferenceOf DriveTrain.getInstance().robotPosition // TODO check math
            val angle = Rotation2d(transform.translation.x.value, transform.translation.y.value, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val angleError = angle + if (isFront) Rotation2d.kZero else Math.PI.radian

            if (angleError.degree.absoluteValue > 45) {
                // plz no disable us when going to loading station, kthx
                this.lastKnownTargetPose = null
            }

            val error = angleError.radian

            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)
            DriveTrain.getInstance().tankDrive(source - turn, source + turn)

            prevError = error
        }
    }

    override fun end() {
        Network.visionDriveActive.setBoolean(false)
        this.lastKnownTargetPose = null
//        ElevatorSubsystem.wantedVisionMode = false
        isActive = false
    }

    override fun interrupted() = end()

    override fun initSendable(builder: SendableBuilder) {

        builder.addDoubleProperty("forwardKp", { kCorrectionKp }, { newP -> kCorrectionKp = newP })
        builder.addDoubleProperty("forwardKd", { kCorrectionKd }, { newD -> kCorrectionKd = newD })

        super.initSendable(builder)
    }

    companion object {
        var kCorrectionKp = 0.8
        var kCorrectionKd = 8.0
        var isActive = false
            private set
    }
}