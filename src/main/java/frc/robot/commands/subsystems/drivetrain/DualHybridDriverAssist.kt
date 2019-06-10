package frc.robot.commands.subsystems.drivetrain

// import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.ghrobotics.vision.JeVoisManager
import frc.robot.Robot
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.LimeLight
import frc.robot.subsystems.superstructure.SuperStructure
// import org.apache.commons.math3.ml.neuralnet.Network
import frc.ghrobotics.vision.TargetTracker
import frc.robot.Network
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.radian

// import org.team5940.pantry.exparimental.command.Command

class DualHybridDriverAssist : DriveTrain.CurvatureDrive() {

    private var lemonLightHasTarget: Boolean = false
    private var lastKnownAngle: Double = 0.0
    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override fun initialize() {
        referencePose = DriveTrain.getInstance().robotPosition
    }

    override fun execute() {

        var isFront = !SuperStructure.getInstance().isPassedThrough

//        isFront = false // for debugging

        var turnInput: Double?

        println("looking for vision targets on the ${if (isFront) "front" else "back"}")

        if (isFront) {
            // it's LimeLight Time
            val lemonLight = LimeLight.getInstance()
            val hasTarget = lemonLight.trackedTargets > 0.5

            this.lemonLightHasTarget = hasTarget

            // check that we have a target
            if (!hasTarget) {
                turnInput = null
                println("no vision targets found!")
            } else {
                println("limelight found vision target!")
                val dx = lemonLight.dx.degree
                turnInput = dx
            }
        } else {

            // is YeeVois Time
            val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, false)

            println("is the back jevois even connected? ${JeVoisManager.isBackJeVoisConnected}")

            val newPose = newTarget?.averagedPose2d
            if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

            val lastKnownTargetPose = this.lastKnownTargetPose
            if (lastKnownTargetPose == null) {
                turnInput = null
                println("no vision targets found!")
            } else {
                println("jevois target found!")
                val transform = lastKnownTargetPose inFrameOfReferenceOf DriveTrain.getInstance().robotPosition // TODO check math
                var angle = Rotation2d(transform.translation.x.value, transform.translation.y.value, true)

                // since it's the back i don't care

                if (angle.degree < -90) angle = angle.plus(180.degree)

                if (angle.degree > 90) angle = angle.minus(180.degree)

                Network.visionDriveAngle.setDouble(angle.degree)
                Network.visionDriveActive.setBoolean(true)

                val angleError = angle + if (isFront) Rotation2d.kZero else Math.PI.radian

//                if(angleError.degree.absoluteValue > 45) {
//                    println("plz no disable us when going to loading station, kthx")
//                    this.lastKnownTargetPose = null
//                    turnInput = null
//                } else {

                turnInput = angleError.degree
//                }
            }
        }

        // check if our vision even sees anything - if not, normal drive time
        if (turnInput == null) {
            println("no target, going to default execute method")
            super.execute()
        } else {

            this.lastKnownAngle = turnInput

            var forward = Robot.m_oi.forwardAxis
            forward *= Math.abs(forward)
            if (DriveTrain.getInstance().cachedGear == DriveTrain.Gear.HIGH) {
                forward *= 0.8
            }

            println("angle error $turnInput")

            println("kp $kp_mutable kd $kd_mutable")

//            kp_mutable = if(!isFront) kJevoiskP else kLemonLightkP
//            kd_mutable = if(!isFront) kJevoiskD else kLemonLightkD

//            var turn = 0.0
            var turn = if (isFront) {
                println("limelightPID")
                kLemonLightkP * turnInput - kLemonLightkD * (turnInput - prevError)
            } else {
                println("jevoisPID")
                kJevoiskP * turnInput - kJevoiskD * (turnInput - prevError)
            }

//            var turn = kp_mutable * turnInput - kd_mutable * (turnInput - prevError)

//            turn /= 100

            if (turn > 0.6) turn = 0.6
            if (turn < -0.6) turn = -0.6

//            turn = Util.limit(turn, 0.5)

//            turn *= -1

//            Network.VisionTab.

//            turn = DriveTrain.getInstance().differentialDrive.getVoltagesFromkV(DifferentialDrive.WheelState(
//                    turn, -turn
//            )).left
//            turn += Math.copySign(0.1, turn)

            println("Commanding state $forward, $turn")

            DriveTrain.getInstance().arcadeDrive(forward, turn)

            prevError = turnInput
        }
    }

    private var kp_mutable = kJevoiskP
    private var kd_mutable = kJevoiskD

    override fun initSendable(builder: SendableBuilder) {

        builder.addDoubleProperty("angle", { Math.toDegrees(lastKnownAngle) }, null)

        builder.addDoubleProperty("kp", { kp_mutable }, {
            this.kp_mutable = it
        })

        builder.addDoubleProperty("kd", { kd_mutable }, {
            this.kd_mutable = it
        })

        super.initSendable(builder)
    }

    companion object {
        const val kJevoiskP = 0.002
        const val kJevoiskD = 0.04
        const val kLemonLightkP = 0.04
        const val kLemonLightkD = 0.0
    }

    override fun isFinished() = false
}
