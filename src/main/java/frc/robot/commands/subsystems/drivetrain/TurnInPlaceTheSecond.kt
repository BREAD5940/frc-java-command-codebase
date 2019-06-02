package frc.robot.commands.subsystems.drivetrain

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.command.Command
import frc.robot.subsystems.DriveTrain
import org.ghrobotics.lib.mathematics.units.Rotation2d

class TurnInPlaceTheSecond(
        private val targetAngle: Rotation2d
) : Command() {

    var lastError = Rotation2d.kZero

    override fun execute() {

        val angle = DriveTrain.getInstance().robotPosition.rotation

        val error = targetAngle - angle

        val pidOutput = (error.radian * kP + (-(error - lastError).radian * kD))

        val demand = DifferentialDrive.ChassisState(
                0.0, pidOutput

        )

        DriveTrain.getInstance().setOutputFromKinematics(demand)

        lastError = error

    }

    override fun isFinished() = (lastError.absoluteValue.degree < 3)

    companion object {
        const val kP = 0.3
        const val kD = 1
    }

}