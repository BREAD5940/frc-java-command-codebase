import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.Scheduler
import frc.robot.lib.command.ParallelRaceGroup
import frc.robot.lib.command.WaitCommand
import org.ghrobotics.lib.mathematics.units.second
import org.junit.Test

class CommandExtensionTest {

    @Test
    fun testParallelRaceGroup() {

        val test1 = TestCommand()

        val testCommand = ParallelRaceGroup(
                test1, TestCommand(), TestCommand()
        )

        assert(!testCommand.isRunning)

        testCommand.start()

        assert(!testCommand.isRunning)

        Scheduler.getInstance().run()

        assert(testCommand.isRunning)

        // trigger 1 command to git commit die
        test1.mIsDone = true

        Scheduler.getInstance().run()

        assert(!testCommand.isRunning)

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

    }



}