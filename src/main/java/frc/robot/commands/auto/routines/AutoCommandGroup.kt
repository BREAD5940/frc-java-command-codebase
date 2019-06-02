package frc.robot.commands.auto.routines

import java.util.ArrayList

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.InstantCommand
import frc.robot.Constants
import frc.robot.commands.subsystems.drivetrain.VisionAssistedTrajectoryTracker
import frc.robot.lib.Logger
import frc.robot.subsystems.DriveTrain
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.map

/**
 * basically just a CommandGroup but with the done() method and time tracking.
 */
open class AutoCommandGroup : CommandGroup() {


    protected fun followVisionAssistedTrajectory(
            originalTrajectory: TimedTrajectory<Pose2dWithCurvature>,
            pathMirrored: BooleanSource,
            radiusFromEnd: Length,
            useAbsoluteVision: Boolean = false
    ) = VisionAssistedTrajectoryTracker(
            pathMirrored.map(originalTrajectory.mirror(), originalTrajectory),
            radiusFromEnd,
            useAbsoluteVision
    )


    protected fun relocalize(position: Pose2d, forward: Boolean, pathMirrored: BooleanSource) = InstantCommand {
        val newPosition = Pose2d(
                pathMirrored.map(position.mirror, position)().translation,
                DriveTrain.getInstance().localization().rotation
        ) + if (forward) Constants.kCenterToForwardIntake else Constants.kBackwardIntakeToCenter
        DriveTrain.getInstance().localization.reset(newPosition)
    }

    fun parallel(vararg commands: Command) = object : AutoCommandGroup() {
        init {
            commands.forEach {
                addParallel(it)
            }
        }
    }

    fun sequential(vararg commands: Command) = object : AutoCommandGroup() {
        init {
            commands.forEach {
                addSequential(it)
            }
        }
    }

    operator fun Command.unaryPlus() {
        addSequential(this)
    }

    internal var start = 0.0

    var commandLog = ArrayList<String>()
        internal set

//    @Synchronized
    override fun start() {
        super.start()
        start = Timer.getFPGATimestamp()
    }

    override fun end() {
        Logger.log("Command " + javaClass.simpleName + " ran in " + (Timer.getFPGATimestamp() - start) + " seconds!")
    }

    fun done(): Boolean {
        return this.isFinished
    }

    @Synchronized
    fun addSequentialLoggable(command: Command, timeout: Double, isReal: Boolean) {
        if (isReal)
            super.addSequential(command, timeout)
        addSequentialLoggable(command, isReal)
    }

    @Synchronized
    fun addSequentialLoggable(command: Command, isReal: Boolean) {
        if (isReal)
            super.addSequential(command)
        logCommand(command, "Sequential")
    }

    @Synchronized
    fun addParallelLoggable(command: Command, timeout: Double, isReal: Boolean) {
        if (isReal)
            super.addParallel(command, timeout)
        addParallelLoggable(command, isReal)
    }

    @Synchronized
    fun addParallelLoggable(command: Command, isReal: Boolean) {
        if (isReal)
            super.addParallel(command)
        logCommand(command, "Parallel")
    }

    @Synchronized
    fun logCommand(command: Command, mode: String) {
        val commandName = command.name
        val commandSubsystem = command.subsystem
        commandLog.add(String.format("Command %s added in %s mode and reserves %s", commandName, mode, commandSubsystem))
    }

}
