package frc.robot.commands.auto.groups;

import org.team5940.pantry.exparimental.command.SequentialCommandGroup;

import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.drivetrain.DrivePower;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PlaceHatch extends SequentialCommandGroup {

	/**
	 * Pickup a hatch from the loading station using some jank open loop code.
	 * Vision code coming soon, I hope
	 * 
	 * @author Matthew Morley
	 */
	public PlaceHatch() {

		addCommands(
				new JankyGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH),
				new FollowVisionTargetTheSecond(4.4),
				new DrivePower(0.4, 0.5),
				new RunIntake(1, 0, 0.5),
				new DrivePower(-.4, 0.5));

	}
}
