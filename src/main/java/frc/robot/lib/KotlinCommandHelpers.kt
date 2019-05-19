package frc.robot.lib

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.Time
import org.team5940.pantry.exparimental.command.*
import org.ghrobotics.lib.utils.BooleanSource

typealias ktRunnable = () -> Unit

@Suppress("unused", "MemberVisibilityCanBePrivate")

/**
 * Decorates this command with a timeout.  If the specified timeout is exceeded before the command
 * finishes normally, the command will be interrupted and un-scheduled.  Note that the
 * timeout only applies to the command returned by this method; the calling command is
 * not itself changed.
 *
 *
 * Note: This decorator works by composing this command within a CommandGroup.  The command
 * cannot be used independently after being decorated, or be re-decorated with a different
 * decorator, unless it is manually cleared from the list of grouped commands with
 * [CommandGroupBase.clearGroupedCommand].  The decorated command can, however, be
 * further decorated without issue.
 *
 * @param seconds the timeout duration
 * @return the command with the timeout added
 */
infix fun Command.withTimeout(seconds: Double): Command {
    return ParallelRaceGroup(this, WaitCommand(seconds))
}

infix fun Command.withTimeout(time: Time) = withTimeout(time.second)

/**
 * Decorates this command with an interrupt condition.  If the specified condition becomes true
 * before the command finishes normally, the command will be interrupted and un-scheduled.
 * Note that this only applies to the command returned by this method; the calling command
 * is not itself changed.
 *
 *
 * Note: This decorator works by composing this command within a CommandGroup.  The command
 * cannot be used independently after being decorated, or be re-decorated with a different
 * decorator, unless it is manually cleared from the list of grouped commands with
 * [CommandGroupBase.clearGroupedCommand].  The decorated command can, however, be
 * further decorated without issue.
 *
 * @param condition the interrupt condition
 * @return the command with the interrupt condition added
 */
infix fun Command.interruptOn(condition: BooleanSource): Command {
    return ParallelRaceGroup(this, WaitUntilCommand(condition))
}

/**
 * Decorates this command with a runnable to run after the command finishes.
 *
 *
 * Note: This decorator works by composing this command within a CommandGroup.  The command
 * cannot be used independently after being decorated, or be re-decorated with a different
 * decorator, unless it is manually cleared from the list of grouped commands with
 * [CommandGroupBase.clearGroupedCommand].  The decorated command can, however, be
 * further decorated without issue.
 *
 * @param toKtRunnable the Runnable to run
 * @return the decorated command
 */
infix fun Command.whenFinished(toRun: ktRunnable): Command {
    return SequentialCommandGroup(this, InstantCommand(Runnable{toRun}))
}

/**
 * Decorates this command with a runnable to run before this command starts.
 *
 *
 * Note: This decorator works by composing this command within a CommandGroup.  The command
 * cannot be used independently after being decorated, or be re-decorated with a different
 * decorator, unless it is manually cleared from the list of grouped commands with
 * [CommandGroupBase.clearGroupedCommand].  The decorated command can, however, be
 * further decorated without issue.
 *
 * @param toKtRunnable the Runnable to run
 * @return the decorated command
 */
infix fun Command.beforeStarting(toRun: ktRunnable): Command {
    return SequentialCommandGroup(InstantCommand(Runnable{toRun}), this)
}

/**
 * Decorates this command with a set of commands to run after it in sequence.  Often more
 * convenient/less-verbose than constructing a new [SequentialCommandGroup] explicitly.
 *
 *
 * Note: This decorator works by composing this command within a CommandGroup.  The command
 * cannot be used independently after being decorated, or be re-decorated with a different
 * decorator, unless it is manually cleared from the list of grouped commands with
 * [CommandGroupBase.clearGroupedCommand].  The decorated command can, however, be
 * further decorated without issue.
 *
 * @param next the commands to run next
 * @return the decorated command
 */
infix fun Command.andThen(next: Command): Command {
    val group = SequentialCommandGroup(this)
    group.addCommands(next)
    return group
}

/**
 * Decorates this command with a set of commands to run parallel to it, ending when the calling
 * command ends and interrupting all the others.  Often more convenient/less-verbose than
 * constructing a new [ParallelDeadlineGroup] explicitly.
 *
 *
 * Note: This decorator works by composing this command within a CommandGroup.  The command
 * cannot be used independently after being decorated, or be re-decorated with a different
 * decorator, unless it is manually cleared from the list of grouped commands with
 * [CommandGroupBase.clearGroupedCommand].  The decorated command can, however, be
 * further decorated without issue.
 *
 * @param parallel the commands to run in parallel
 * @return the decorated command
 */
infix fun Command.deadlineWith(parallel: Command): Command {
    return ParallelDeadlineGroup(this, parallel)
}

/**
 * Decorates this command with a set of commands to run parallel to it, ending when the last
 * command ends.  Often more convenient/less-verbose than constructing a new
 * [ParallelCommandGroup] explicitly.
 *
 *
 * Note: This decorator works by composing this command within a CommandGroup.  The command
 * cannot be used independently after being decorated, or be re-decorated with a different
 * decorator, unless it is manually cleared from the list of grouped commands with
 * [CommandGroupBase.clearGroupedCommand].  The decorated command can, however, be
 * further decorated without issue.
 *
 * @param parallel the commands to run in parallel
 * @return the decorated command
 */
infix fun Command.alongWith(parallel: Command): Command {
    val group = ParallelCommandGroup(this)
    group.addCommands(parallel)
    return group
}

/**
 * Decorates this command with a set of commands to run parallel to it, ending when the first
 * command ends.  Often more convenient/less-verbose than constructing a new
 * [ParallelRaceGroup] explicitly.
 *
 *
 * Note: This decorator works by composing this command within a CommandGroup.  The command
 * cannot be used independently after being decorated, or be re-decorated with a different
 * decorator, unless it is manually cleared from the list of grouped commands with
 * [CommandGroupBase.clearGroupedCommand].  The decorated command can, however, be
 * further decorated without issue.
 *
 * @param parallel the commands to run in parallel
 * @return the decorated command
 */
infix fun Command.raceWith(parallel: Command): Command {
    val group = ParallelRaceGroup(this)
    group.addCommands(parallel)
    return group
}

public val Number.meterToInch : Double
    get() = this.toDouble() / SILengthConstants.kInchToMeter

public val Number.meterToFoot : Double
    get() = this.toDouble() / SILengthConstants.kFeetToMeter


///**
// * Return a sequential command group of this and then the other command
// */
//operator fun ParallelCommandGroup.unaryPlus(command: Command): Command {
//    return this.alongWith(command)
//}
//
///**
// * Return a sequential command group of this and then the other command
// */
//operator fun SequentialCommandGroup.unaryPlus(command: Command): Command {
//    return this.andThen(command)
//}
//
///**
// * Return a sequential command group of this and then the other command
// */
//fun CommandGroupBase.unaryPlus(command: Command) {
//    this.addCommands(command)
//}