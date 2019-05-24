package frc.robot.lib.command

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import org.ghrobotics.lib.mathematics.units.Time
import java.util.Vector

@Suppress("unused")
class ParallelRaceGroup(vararg command: Command) : CommandGroup() {

    init {
        addCommands(*command)
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

    override fun isFinished(): Boolean {

//        val reflectionGang = this.javaClass.getDeclaredMethod("isFinished").invoke(javaClass) as Boolean

//        kotlin.runCatching {  }

        return try {
            super.isFinished() || javaClass.getDeclaredField("isFinished").let {
                it.isAccessible = true
                val value = it.getBoolean(this)
                //todo
                return@let value;
            }
        } catch (e: Throwable) {
            super.isFinished()
        }

    }

}

@Suppress("unused")
class WaitCommand(duration: Time) : Command() {

    protected var m_timer = Timer()
    private var m_duration: Double = -1.0

    init {
        setRunWhenDisabled(true)
    }

    public override fun initialize() {
        m_timer.reset()
        m_timer.start()
    }

    override fun end() {
        m_timer.stop()
    }

    override fun interrupted() = end()

    public override fun isFinished(): Boolean {
        return m_timer.hasPeriodPassed(m_duration)
    }

}