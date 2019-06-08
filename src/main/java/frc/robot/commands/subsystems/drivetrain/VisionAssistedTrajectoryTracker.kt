package frc.robot.commands.subsystems.drivetrain

import edu.wpi.first.wpilibj.command.Command
import frc.ghrobotics.vision.TargetTracker
import frc.robot.Constants
import frc.robot.Network
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.LimeLight
//import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
//import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput
import org.ghrobotics.lib.utils.Source
import kotlin.math.PI

/**
 * Command to follow a smooth trajectory using a trajectory following controller
 *
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class VisionAssistedTrajectoryTracker(
        val trajectorySource: Source<Trajectory<Time, TimedEntry<Pose2dWithCurvature>>>,
        val radiusFromEnd: Length,
        val useAbsoluteVision: Boolean = false,
        val useLimeLightOverTargetTracker: Boolean = true
) : Command() {

    private var trajectoryFinished = false

    var limelightHasTarget = false
    var limeLightAngle: Rotation2d? = null

    private var prevError = 0.0 // cached error for the PD loop

    @Suppress("LateinitUsage")
    private lateinit var trajectory: Trajectory<Time, TimedEntry<Pose2dWithCurvature>>

    override fun isFinished() = trajectoryFinished
    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override fun initialize() {
        trajectory = trajectorySource()
        DriveTrain.getInstance().trajectoryTracker.reset(trajectory)
        trajectoryFinished = false
        LiveDashboard.isFollowingPath = true
    }

    private var lastKnownTargetPose: Pose2d? = null // cache of the vision target pose

    override fun execute() {
//        val robotPositionWithIntakeOffset = IntakeSubsystem.robotPositionWithIntakeOffset

        val robotPosition = DriveTrain.getInstance().robotPosition
        
        // get the next state from the trajectory tracker
        val nextState = DriveTrain.getInstance().trajectoryTracker.nextState(
                robotPosition)

        // check if we are close enough to the last pose to engage vision
        val withinVisionRadius =
                robotPosition.translation.distance(
                        trajectory.lastState.state.pose.translation
                ) < radiusFromEnd.value

        // if we are, try to find a new target
        if (withinVisionRadius) {

            if(!useLimeLightOverTargetTracker && trajectory.reversed) {

                val newTarget = if (useAbsoluteVision) {
                    TargetTracker.getAbsoluteTarget((trajectory.lastState.state.pose + Constants.kCenterToForwardIntake).translation)
                } else {
                    TargetTracker.getBestTarget(!trajectory.reversed) // usually this is the case
                }

                val newPose = newTarget?.averagedPose2d

                // store this pose if it's for real
                if (newTarget?.isAlive == true && newPose != null) this.lastKnownTargetPose = newPose
            } else if(useLimeLightOverTargetTracker) {

                // we COULD use the limelight
                this.limelightHasTarget = LimeLight.getInstance().trackedTargets > 0

                this.limeLightAngle = if(limelightHasTarget) {
                    LimeLight.getInstance().dx + robotPosition.rotation}
                else null

            }

        }

        val lastKnownTargetPose = this.lastKnownTargetPose

        if (lastKnownTargetPose != null || limeLightAngle != null) {
            println("VISION ACTIVE")
            visionActive = true

            var error: Double
            var turn: Double
            var youShouldUseLimeLight = false

            if(limeLightAngle != null && useLimeLightOverTargetTracker) {

                error = -(limeLightAngle!! - robotPosition.rotation).radian

                youShouldUseLimeLight = true

                Network.visionDriveAngle.setDouble(Math.toDegrees(error))
                Network.visionDriveActive.setBoolean(true)

                println("limelight angle error: $error")

                turn = kLimeLightKp * error + kLimeLightKd * (error - prevError)

            } else if(lastKnownTargetPose != null) {

                // find our angle to the target
                val transform = lastKnownTargetPose inFrameOfReferenceOf robotPosition
                val angle = Rotation2d(transform.translation.x.meter, transform.translation.y.meter, true)

                Network.visionDriveAngle.setDouble(angle.degree)
                Network.visionDriveActive.setBoolean(true)

                error = (angle + if (!trajectory.reversed) Rotation2d.kZero else Math.PI.radian).radian

                println("jevois angle error: $error")

                turn = kJevoisKp * error + kJevoisKd * (error - prevError)

            } else {
                println("NO TARGET FOUND, mega prank (you should never see this...), returning...")
                DriveTrain.getInstance().setOutput(nextState) // go back to RAMSETE tracking mode
                return
            }

            val newCommandedOutput = TrajectoryTrackerOutput(
                    nextState.linearVelocity,
                    0.meter.acceleration,
                    turn.radian.velocity,
                    0.radian.acceleration
            )

            println("demanding output with turn ${turn.radian.velocity.value}")

            // set the drivetrain to the RAMSETE/whatever linear velocity and the PD loop's output for turn
            DriveTrain.getInstance().setOutput(
                    newCommandedOutput
            )

            prevError = error // save error for the PD loop

        } else { // just do the boring Ramsete stuff
            DriveTrain.getInstance().setOutput(nextState)
        }

        // update LiveDashboard
        val referencePoint = DriveTrain.getInstance().trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x.feet
            LiveDashboard.pathY = referencePose.translation.y.feet
            LiveDashboard.pathHeading = referencePose.rotation.radian
        }

        trajectoryFinished = DriveTrain.getInstance().trajectoryTracker.isFinished
    }

    /**
     * Make sure that the drivetrain is stopped at the end of the command.
     */
    override fun end() {
        DriveTrain.getInstance().zeroOutputs()
        LiveDashboard.isFollowingPath = false
        visionActive = false
    }

    override fun interrupted() = end()

    companion object {
        const val kJevoisKp = 5.5 * (2* PI) / 360.0
        const val kJevoisKd = 0.0

        const val kLimeLightKp = 0.2 * 3.5
        const val kLimeLightKd = 0.toDouble()

        var visionActive = false

    }
}