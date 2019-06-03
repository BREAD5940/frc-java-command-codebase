package frc.robot.lib.command

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import java.util.Vector

//@Suppress("unused")
//fun Command.withTimeout(runtime: Time) = ParallelRaceGroup(this, WaitCommand(runtime))

@Suppress("unused")
class AutonomousCommand : CommandGroup() {

    var interruptCondition = {false}

    override fun isFinished(): Boolean {
        return super.isFinished() || interruptCondition() || (if(mDuration > 0) mTimer.hasPeriodPassed(mDuration) else false)
    }

    fun withTimeout(duration: Time) {
        mDuration = duration.second
    }

    override fun initialize() {

        if(mDuration > 0) mTimer.reset() ; mTimer.start()

        super.initialize()
    }

    private var mTimer = Timer()
    private var mDuration: Double = -1.0

}


@Suppress("unused")
class WaitCommand(val duration: Time) : Command() {

    constructor(duration: Double) : this(duration.second)

    private var mTimer = Timer()
    private var mDuration: Double = -1.0

    init {
        setRunWhenDisabled(true)
        mDuration = duration.second
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
//        println(mTimer.hasPeriodPassed(mDuration))
        return mTimer.hasPeriodPassed(mDuration)
    }

}