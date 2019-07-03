package frc.robot.commands.auto

import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.robot.Network
import frc.robot.commands.auto.paths.TrajectoryWaypoints
import frc.robot.commands.auto.routines.offseasonRoutines.BottomRocketRoutine
import frc.robot.commands.auto.routines.offseasonRoutines.CargoShipRoutine
import frc.robot.commands.auto.routines.offseasonRoutines.TestRoutine
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.Source

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous {

//    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    private val JUST = Network.autoModeChooser

    val options = HashMap<Mode, Command>()

    init {

        options[Autonomous.Mode.NOTHING] = InstantCommand()
        options[Autonomous.Mode.LEFT_TO_ROCKET] = BottomRocketRoutine(true)
        options[Autonomous.Mode.LEFT_TO_CARGO_SHIP_FRONT] = CargoShipRoutine(CargoShipRoutine.Mode.FRONT, true)
        options[Autonomous.Mode.LEFT_TO_CARGO_SHIP_SIDE] = CargoShipRoutine(CargoShipRoutine.Mode.SIDE, true)
        options[Mode.TEST_ROUTINE] = TestRoutine()

        options[Autonomous.Mode.RIGHT_TO_ROCKET] = BottomRocketRoutine(false)
        options[Autonomous.Mode.RIGHT_TO_CARGO_SHIP_FRONT] = CargoShipRoutine(CargoShipRoutine.Mode.FRONT, false)
        options[Autonomous.Mode.RIGHT_TO_CARGO_SHIP_SIDE] = CargoShipRoutine(CargoShipRoutine.Mode.SIDE, false)
        options[Autonomous.Mode.CENTER_TO_CARGO_SHIP_LEFT] = CargoShipRoutine(CargoShipRoutine.Mode.FRONT, true)
        options[Autonomous.Mode.CENTER_TO_CARGO_SHIP_RIGHT] = CargoShipRoutine(CargoShipRoutine.Mode.FRONT, false)

        options.forEach {
            println("adding ${it.key} to the auto chooser")
            JUST.addOption(it.key.toString(), it.key)
        }
        JUST.setDefaultOption("NOTHING", Autonomous.Mode.NOTHING)
    }

    @Suppress("LocalVariableName")
    private val IT = ""

//    private val startingPositionMonitor = startingPosition.monitor
//    private val isReadyMonitor = isReady.monitor
//    private val modeMonitor = { Robot.lastRobotMode }.monitor

    fun JUSTS3NDIT() = JUST S3ND IT

    fun JUSTSTOP() = JUST STOP IT

    enum class StartingPositions(val pose: Pose2d) {
        LEFT(TrajectoryWaypoints.kSideStart.mirror),
        CENTER(TrajectoryWaypoints.kCenterStart),
        RIGHT(TrajectoryWaypoints.kSideStart)
    }

    enum class Mode { NOTHING, LEFT_TO_ROCKET, LEFT_TO_CARGO_SHIP_FRONT, LEFT_TO_CARGO_SHIP_SIDE,
        RIGHT_TO_ROCKET, RIGHT_TO_CARGO_SHIP_FRONT, RIGHT_TO_CARGO_SHIP_SIDE, CENTER_TO_CARGO_SHIP_LEFT,
        CENTER_TO_CARGO_SHIP_RIGHT, TEST_ROUTINE
    }

    infix fun SendableChooser<Mode>.S3ND(it: Any) {

        val selection = this@S3ND.selected

        val toStart = options[selection]

        toStart?.start()
    }

    private infix fun SendableChooser<Mode>.STOP(it: Any) {
        val selection = options[selected]
        selection?.cancel()
    }
}
