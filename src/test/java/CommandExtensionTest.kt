import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.Scheduler
//import frc.robot.lib.command.ParallelDeadlineGroup
//import frc.robot.lib.command.ParallelRaceGroup
import frc.robot.lib.command.WaitCommand
import org.ghrobotics.lib.mathematics.units.second
import org.junit.Test

class CommandExtensionTest {

//    @Test
//    fun testParallelRaceGroup() {
//
//        val test1 = TestCommand()
//
//        val testCommand = ParallelRaceGroup(
//                test1, TestCommand(), TestCommand()
//        )
//
//        assert(!testCommand.isRunning)
//
//        testCommand.start()
//
//        assert(!testCommand.isRunning)
//
//        Scheduler.getInstance().run()
//
//        assert(testCommand.isRunning)
//
//        // trigger 1 command to git commit die
//        test1.mIsDone = true
//
//        Scheduler.getInstance().run()
//
//        assert(!testCommand.isRunning)
//
//    }

    @Test
    fun testParallelCommandGroup() {

        val test1 = TestCommand()
        val test2 = TestCommand()

        val group = testGroup(
              test1, test2
        )

        Scheduler.getInstance().run()
        Scheduler.getInstance().run()

        println("test 1 initilized? ${test1.hasBeenInitilized}")

        println("group started? ${group.isRunning}")
        println("group done? ${Scheduler.getInstance().subsystem}")

    }

    private class testGroup(vararg commands: Command) : CommandGroup() {
        init {
            commands.forEach {
                super.addParallel(it)
            }
        }
    }

//    @Test
//    fun testParallelDeadlineGroup() {
//
//        val test1 = TestCommand()
//        val test2 = TestCommand()
//        val test3 = TestCommand()
//        val test4 = TestCommand()
//
//        val testCommand = ParallelDeadlineGroup(
//                test1, test2, test3, test4
//        )
//
//        assert(!testCommand.isRunning)
//        assert(!test1.isRunning)
//
//        testCommand.start()
//
//        assert(!test1.isRunning)
//        assert(!testCommand.isRunning)
//
//        Scheduler.getInstance().run()
//
//        println("big boi group running? ${testCommand.isRunning}")
//        assert(testCommand.isRunning)
//        println("subcommand execute called? ${test1.hasBeenExecuted}")
////        assert(test1.hasBeenExecuted) // y u fail on meeeeeeeeeeeeeeeeeeee
////        assert(test1.executeCount > 0)
//
//        // trigger 1 command to git commit die
//        test1.mIsDone = true
//
//        Scheduler.getInstance().run()
//
//        assert(!testCommand.isRunning)
//
//    }

    private fun getBooleanMethodViaReflection(object_: Any , methodName: String) : Boolean {
        return try {
            object_.javaClass.getDeclaredField(methodName).let {
                it.isAccessible = true
                val value = it.getBoolean(object_)
                //todo
                return@let value;
            }
        } catch (e: Throwable) {
            false
        }
    }

    @Test
    fun testWaitCommand() {

        val command = WaitCommand(0.5.second)

        Scheduler.getInstance().run()
        assert(!command.isRunning)
        command.start()
        Scheduler.getInstance().run()
        assert(command.isRunning)

        Scheduler.getInstance().run()
        Thread.sleep(0.25.second.millisecond.toLong())
        assert(command.isRunning)

        Scheduler.getInstance().run()
        Thread.sleep(0.35.second.millisecond.toLong())
        Scheduler.getInstance().run()
        assert(!command.isRunning)

    }

    class TestCommand : Command() {
        override fun isFinished(): Boolean = mIsDone

        var mIsDone = false

        var hasBeenExecuted = false
        var executeCount = 0

        var hasBeenInitilized = false

        override fun initialize() {
            hasBeenInitilized = true
        }

        override fun execute() {
            println("executing!")
            hasBeenExecuted = true
            executeCount += 1
        }

    }



}