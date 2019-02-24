package frc.robot.commands.auto.routines;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceTheSecond;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTarget;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PickupHatch extends CommandGroup {

	/**
	 * Add your docs here.
	 */
	public PickupHatch() {

		addParallel(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal), iPosition.HATCH));

		addSequential(new FollowVisionTarget(1, 6, 10));

		addSequential(new RunIntake(-1, 0.5));

		addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(.5), true));
	}
}
