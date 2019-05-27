import frc.robot.lib.command.AutoCommandGroup
//import frc.robot.lib.command.TestGroup
import org.junit.Test
import org.team5940.pantry.exparimental.command.Command
import org.team5940.pantry.exparimental.command.CommandScheduler
import org.team5940.pantry.exparimental.command.SendableCommandBase
import org.team5940.pantry.exparimental.command.SendableSubsystemBase

class CommandExtensionTest {

    class Subsystem: SendableSubsystemBase() {}

    @Test
    fun testAutoCommandGroup() {

        val subsystem1 = Subsystem()
        val subsystem2 = Subsystem()

        val command1 = VerboseCommand()
        val command2 = VerboseCommand()
        command1.name = "command1"
        command2.name = "command2"
        command1.addRequirements(subsystem1)
        command2.addRequirements(subsystem2)

        println("command 1's name is ${command1.name}")
        assert(command1.name == "command1")

        val group = TestGroup(command1, command2)
        group.name = "BigChungus"


        CommandScheduler.getInstance().run()

        assert(!group.isScheduled)

        group.schedule()

        CommandScheduler.getInstance().run()

//        assert(group.isScheduled)

    }

    class VerboseCommand : SendableCommandBase() {

//        var myName = javaClass.simpleName

        override fun initialize() {
            println("initilizing command $name")
        }

        override fun execute() {
            println("executing command $name")
        }

        override fun end(interrupted: Boolean) {
            println("ending command $name")
        }

        override fun getName() = m_name

    }

    class TestGroup(command1: VerboseCommand, command2: VerboseCommand) : AutoCommandGroup() {

        init {
            +sequence(command1, command2)
//            addCommands()
//            literallyAddCommands(command1)
//            literallyAddCommands(command2)
        }

        fun literallyAddCommands(other: VerboseCommand) {

            println("adding command ${other.name} to $m_name")
            addCommands(other)

        }

    }

}