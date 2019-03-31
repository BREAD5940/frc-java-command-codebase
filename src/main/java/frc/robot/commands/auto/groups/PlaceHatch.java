package frc.robot.commands.auto.groups;

import org.team5940.pantry.experimental.command.RunCommand;
import org.team5940.pantry.experimental.command.SequentialCommandGroup;

import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
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
				(new JankyGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH)),
				(new FollowVisionTargetTheSecond(4.4)),

				new RunCommand(() -> {
					DriveTrain.getInstance().arcadeDrive(0.4, 0, false);
				}, DriveTrain.getInstance()).deadlineWith(
						new RunCommand(
								() -> {
									Intake.getInstance().setHatchSpeed(1);
								}))
						.withTimeout(0.5));

	}
}
