package frc.robot.commands.subsystems.drivetrain

import java.util.function.BooleanSupplier
import java.util.function.Supplier

import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput

import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import frc.robot.Robot
import frc.robot.commands.auto.routines.yeOldeRoutines.Trajectories
import frc.robot.lib.AutoCommand
import frc.robot.lib.Logger
import frc.robot.subsystems.DriveTrain
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.second

// @SuppressWarnings({"WeakerAccess", "unused"})
class TrajectoryTrackerCommand(private val driveBase: DriveTrain, private val trajectoryTracker: TrajectoryTracker,
                               private val trajectorySource: Supplier<TimedTrajectory<Pose2dWithCurvature>>,
                               private val shouldMirrorPath: BooleanSupplier, private val reset: Boolean)
    : AutoCommand() {
    private var output: TrajectoryTrackerOutput? = null
    // TODO make sure that this fabled namespace collision doesn't happen on Shuffleboard
    internal var mDesiredLeft: Length? = null
    internal var mDesiredRight: Length? = null
    internal var mCurrentLeft: Double = 0.toDouble()
    internal var mCurrentRight: Double = 0.toDouble()

    // private NetworkTableEntry refVelEntry = Shuffleboard.getTab("Auto").getLayout("List", "Pathing info").add("Reference Velocity", 0).getEntry();
    // private NetworkTableEntry currentVelEntry = Shuffleboard.getTab("Auto").getLayout("List", "Pathing info").add("Current Velocity", 0).getEntry();

    private var mUpdateNotifier: Notifier? = null

    val trajectory: TimedTrajectory<Pose2dWithCurvature>
        get() = this.trajectorySource!!.get()

//    @JvmOverloads
//    constructor(driveBase: DriveTrain, trajectorySource: Supplier<TimedTrajectory<Pose2dWithCurvature>>, reset: Boolean = false) : this(driveBase, Robot.drivetrain.getTrajectoryTracker(), trajectorySource, { false }, reset) {
//    }

    init {
        requires(driveBase)
        // this.mModel = driveBase.getDifferentialDrive();
    }

    override fun initialize() {
        LiveDashboard.isFollowingPath = false

        var trajectory = this.trajectorySource.get()

        if (shouldMirrorPath.asBoolean) {
            trajectory = trajectory.mirror()
        }

        Logger.log("get: " + trajectorySource.get().firstState.state.curvature.toString())

        trajectoryTracker.reset(trajectory)

        Logger.log("first pose: " + trajectorySource.get().firstState.state.pose.translation.x.feet)

        if (reset) {
            Robot.drivetrain.localization.reset(trajectorySource.get().firstState.state.pose)
        }

        // Logger.log("desired linear, real linear");

        LiveDashboard.isFollowingPath = true

        mUpdateNotifier = Notifier {
            output = trajectoryTracker.nextState(driveBase.robotPosition, Timer.getFPGATimestamp().second)

            val referencePoint = trajectoryTracker.referencePoint
            if (referencePoint != null) {
                val (translation, rotation) = referencePoint.state.state.pose

                LiveDashboard.pathX = translation.x.feet
                LiveDashboard.pathY = translation.y.feet
                LiveDashboard.pathHeading = rotation.radian

                // Shuffleboard.getTab("Auto").getLayout("Pathing", BuiltInLayouts.kList).

                // refVelEntry.setDouble(referencePoint.getState().getVelocity().getType$FalconLibrary().getFeet());
                // currentVelEntry.setDouble(driveBase.getLeft().getFeetPerSecond());

            }
            // Logger.log("Linear: " + output.getLinearVelocity().getValue() + " Angular: " + output.getAngularVelocity().getValue() );
            driveBase.setOutput(output!!)

        }
        mUpdateNotifier!!.startPeriodic(0.01)
    }

    override fun end() {
        mUpdateNotifier?.stop()
        driveBase.stop()
        LiveDashboard.isFollowingPath = false
    }

    override fun isFinished()= trajectoryTracker.isFinished

}
