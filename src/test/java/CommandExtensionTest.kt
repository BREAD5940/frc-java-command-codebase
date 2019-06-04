//import edu.wpi.first.wpilibj.command.Command
//import edu.wpi.first.wpilibj.command.CommandGroup
//import edu.wpi.first.wpilibj.command.Scheduler
////import frc.robot.lib.command.ParallelDeadlineGroup
////import frc.robot.lib.command.ParallelRaceGroup
//import frc.robot.lib.command.WaitCommand
//import org.ghrobotics.lib.mathematics.units.millisecond
//import org.ghrobotics.lib.mathematics.units.second
//import org.junit.Test
//import org.team5940.pantry.exparimental.command.CommandScheduler
//
//class CommandExtensionTest {
//
////    @Test
////    fun testParallelRaceGroup() {
////
////        val test1 = TestCommand()
////
////        val testCommand = ParallelRaceGroup(
////                test1, TestCommand(), TestCommand()
////        )
////
////        assert(!testCommand.isRunning)
////
////        testCommand.start()
////
////        assert(!testCommand.isRunning)
////
////        Scheduler.getInstance().run()
////
////        assert(testCommand.isRunning)
////
////        // trigger 1 command to git commit die
////        test1.mIsDone = true
////
////        Scheduler.getInstance().run()
////
////        assert(!testCommand.isRunning)
////
////    }
//
//    @Test
//    fun testTestCommand() {
//        val test = TestCommand()
//        assert(!test.mIsDone)
//        assert(!getBooleanMethodViaReflection(test, "isFinished"))
//        assert(!test.hasBeenExecuted)
//        assert(!test.hasBeenInitilized)
//        test.start()
//        assert(!test.isCompleted)
//        CommandScheduler.getInstance().run()
//        CommandScheduler.getInstance().run()
//        assert(!test.mIsDone)
//        assert(!getBooleanMethodViaReflection(test, "isFinished"))
////        assert(test.hasBeenExecuted)
////        assert(test.hasBeenInitilized)
//        assert(!test.isCompleted)
//        test.mIsDone = true
//        CommandScheduler.getInstance().run()
//        assert(test.isCompleted)
//
//    }
//
//    @Test
//    fun testAutoCommandGroup() {
//
//        val test1 = TestCommand()
//        val test2 = TestCommand()
//
//        class AutoTestCommand: CommandGroup() {
//            init {
//                addSequential(test1)
//                addSequential(test2)
//            }
//        }
//
//        val command = AutoTestCommand()
//
//        CommandScheduler.getInstance().run()
//        Thread.sleep(20)
//        command.start()
//        CommandScheduler.getInstance().run()
//        Thread.sleep(20)
//        CommandScheduler.getInstance().run()
//        Thread.sleep(20)
//        CommandScheduler.getInstance().run()
//        Thread.sleep(20)
//        println("command running? ${command.isRunning}")
//
//
//    }
//
//
//
//    @Test
//    fun testParallelCommandGroup() {
//
//        val test1 = TestCommand()
//        val test2 = TestCommand()
//
//        val group = testGroup(
//              test1, test2
//        )
//
//        Scheduler.getInstance().run()
//        Scheduler.getInstance().run()
//
//        println("test 1 initilized? ${test1.hasBeenInitilized}")
//
//        println("group started? ${group.isRunning}")
//        println("GROUP done? ${getBooleanMethodViaReflection(group, "isFinished")}")
//
//    }
//
//    private class testGroup(vararg commands: Command) : CommandGroup() {
//        init {
//            commands.forEach {
//                super.addParallel(it)
//            }
//        }
//    }
//
////    @Test
////    fun testParallelDeadlineGroup() {
////
////        val test1 = TestCommand()
////        val test2 = TestCommand()
////        val test3 = TestCommand()
////        val test4 = TestCommand()
////
////        val testCommand = ParallelDeadlineGroup(
////                test1, test2, test3, test4
////        )
////
////        assert(!testCommand.isRunning)
////        assert(!test1.isRunning)
////
////        testCommand.start()
////
////        assert(!test1.isRunning)
////        assert(!testCommand.isRunning)
////
////        Scheduler.getInstance().run()
////
////        println("big boi group running? ${testCommand.isRunning}")
////        assert(testCommand.isRunning)
////        println("subcommand execute called? ${test1.hasBeenExecuted}")
//////        assert(test1.hasBeenExecuted) // y u fail on meeeeeeeeeeeeeeeeeeee
//////        assert(test1.executeCount > 0)
////
////        // trigger 1 command to git commit die
////        test1.mIsDone = true
////
////        Scheduler.getInstance().run()
////
////        assert(!testCommand.isRunning)
////
////    }
//
//    private fun getBooleanMethodViaReflection(object_: Any , methodName: String) : Boolean {
//        return try {
//            object_.javaClass.getDeclaredMethod(methodName).let {
//                it.isAccessible = true
//                val value = it.invoke(it)
//                //todo
//                return@let value as Boolean;
//            }
//        } catch (e: Throwable) {
//            false
//        }
//    }
//
//    @Test
//    fun testWaitCommand() {
//
//        val command = WaitCommand(0.5.second)
//
//        Scheduler.getInstance().run()
//        assert(!command.isRunning)
//        command.start()
//        Scheduler.getInstance().run()
//        assert(command.isRunning)
//
//        Scheduler.getInstance().run()
//        Thread.sleep(0.25.second.millisecond.toLong())
//        assert(command.isRunning)
//
//        Scheduler.getInstance().run()
//        Thread.sleep(0.35.second.millisecond.toLong())
//        Scheduler.getInstance().run()
//        assert(!command.isRunning)
//
//    }
//
//    class TestCommand : Command() {
//        override fun isFinished(): Boolean {
//            println("isFinished? returning $mIsDone")
//            return mIsDone
//        }
//
//        var mIsDone = false
//
//        var hasBeenExecuted = false
//        var executeCount = 0
//
//        var hasBeenInitilized = false
//
//        override fun initialize() {
//            println("command now initializing")
//            hasBeenInitilized = true
//        }
//
//        override fun execute() {
//            println("executing!")
//            hasBeenExecuted = true
//            executeCount += 1
//        }
//
//    }
//
//
//
//}