package frc.robot.lib.command

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.map
import java.util.Vector

@Suppress("unused")
class ParallelRaceGroup() : CommandGroup() {

    constructor(vararg commands: Command) : this() {
        addCommands(*commands)
    }

    private val commandVector: Vector<Command> = Vector()

    fun addCommands(vararg commands: Command) {
        commands.forEach {
            this.addParallel(it)
            commandVector.addElement(it)
        }
    }

    override fun isFinished(): Boolean {
//        val commands = m_commands
        var toReturn = super.isFinished()
        commandVector.forEach {
            toReturn = toReturn || it.isCompleted
        }
        return toReturn
    }

}

@Suppress("unused")
class AutonomousCommand : CommandGroup() {

    var interruptCondition = {false}

    fun withTimeout(runtime: Time) = ParallelRaceGroup(this, WaitCommand(runtime))

    override fun isFinished(): Boolean {
        return super.isFinished() || interruptCondition()
    }

}

@Suppress("unused")
class ParallelDeadlineGroup(val deadline: Command, vararg commands: Command) : CommandGroup() {

    init {
        this.addParallel(deadline)
        commands.forEach {
            this.addParallel(it)
        }
    }

    var startedRunning = false

    override fun execute() {

        if(!startedRunning && deadline.isRunning) startedRunning = true

        super.execute()
    }

    override fun isFinished(): Boolean {

        return super.isFinished() || deadline.isRunning

    }

}

@Suppress("unused")
class WaitCommand(duration: Time) : Command() {

    private var mTimer = Timer()
    private var mDuration: Double = -1.0

    init {
        setRunWhenDisabled(true)
    }

    public override fun initialize() {
        mTimer.reset()
        mTimer.start()
    }

    override fun end() {
        mTimer.stop()
    }

    override fun interrupted() = end()

    public override fun isFinished(): Boolean {
        return mTimer.hasPeriodPassed(mDuration)
    }

}