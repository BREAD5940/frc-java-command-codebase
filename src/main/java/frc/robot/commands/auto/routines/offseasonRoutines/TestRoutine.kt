package frc.robot.commands.auto.routines.offseasonRoutines

import edu.wpi.first.wpilibj.command.InstantCommand
import frc.robot.RobotConfig
import frc.robot.commands.auto.paths.TrajectoryFactory
import frc.robot.commands.auto.routines.AutoRoutine
import frc.robot.commands.subsystems.drivetrain.DrivePower
import frc.robot.commands.subsystems.superstructure.JankyGoToState
import frc.robot.commands.subsystems.superstructure.RunIntake
import frc.robot.subsystems.DriveTrain
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second

class TestRoutine: AutoRoutine() {

    init {

        +InstantCommand{
            DriveTrain.getInstance().localization.reset(TrajectoryFactory.tenFootTest.firstState.state.pose)
        }

//        +JankyGoToState(RobotConfig.auto.fieldPositions.hatchLowGoal, SuperStructure.iPosition.HATCH)


        +parallel(

                followVisionAssistedTrajectory(TrajectoryFactory.tenFootTest, { false }, 4.feet, false),

                RunIntake.grabHatchFor(TrajectoryFactory.tenFootTest.duration)

        )

        +parallel(
                DrivePower(-0.3, 1.0),
                RunIntake.grabHatchFor(0.5.second)
        )

    }

}