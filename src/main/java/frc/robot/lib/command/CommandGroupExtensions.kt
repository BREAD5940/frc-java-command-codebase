package frc.robot.lib.command

import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.BooleanSource
import org.team5940.pantry.exparimental.command.*
import java.util.Vector

//@Suppress("unused")
//fun Command.withTimeout(runtime: Time) = ParallelRaceGroup(this, WaitCommand(runtime))

@Suppress("unused")
abstract class AutoCommandGroup : SequentialCommandGroup() {

    infix fun withExit(exitCondition: BooleanSource) =
            ParallelRaceGroup(this, WaitUntilCommand(exitCondition))

    infix fun withTimeout(time: Time) =
            ParallelRaceGroup(this, WaitCommand(time.second))

//    val myName = "autoCommandGroup"
    init {
        name = "AutoGroupGanggg"
    }

    // TODO check implementation
    operator fun CommandGroupBase.unaryPlus() {
        println("adding command  with requirements ${this.requirements} to $m_name!")
        addCommands(this)
    }

    override fun getName(): String = m_name

}