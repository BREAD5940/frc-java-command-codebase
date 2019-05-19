package frc.robot.lib

import edu.wpi.first.wpilibj.Sendable
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard

object ShuffleBoardTabs {

    val Subsystems = Shuffleboard.getTab("Subsystems")

    val Auto = Shuffleboard.getTab("Auto")

    var index = 0

    fun registerSubsystem(subsystem: Sendable) {

        Subsystems.add(subsystem).withPosition(index,0).withSize(2,6)

        index += 2

    }

    val Vision = Shuffleboard.getTab("Camera")

}