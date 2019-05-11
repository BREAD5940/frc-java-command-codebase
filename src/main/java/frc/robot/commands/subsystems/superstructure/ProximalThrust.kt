package frc.robot.commands.subsystems.superstructure

import edu.wpi.first.wpilibj.command.Command
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d

class ProximalThrust(
        private val structure: SuperStructure,
        private val elevator : Elevator,
        private val pokeDistance: Double,
        private val proximalLen: Double
) : Command() {

    init {
        requires(structure)
        requires(structure.elbow)
        requires(structure.wrist)
        requires(elevator)
    }

    lateinit var startLocation: Pose2d

    override fun initialize() {

    }

    override fun isFinished(): Boolean {
        return false
    }

}