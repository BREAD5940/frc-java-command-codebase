package frc.robot.commands.auto.groups;

import org.team5940.pantry.experimental.command.RunCommand;
import org.team5940.pantry.experimental.command.SequentialCommandGroup;
import org.team5940.pantry.experimental.command.StartEndCommand;

import frc.robot.commands.subsystems.drivetrain.DrivePowerAndIntake;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.HatchMechState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class GrabHatch extends SequentialCommandGroup {

	/**
	 * Pickup a hatch from the loading station using some jank open loop code.
	 * Vision code coming soon, I hope
	 * 
	 * @author Matthew Morley
	 */
	public GrabHatch() {

		super(
				// new SetHatchMech(HatchMechState.kClamped),
				new RunCommand(() -> {
					Intake.getInstance().setHatchMech(HatchMechState.kClamped);
				}, Intake.getInstance()),
				// yes.addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
				new JankyGoToState(iPosition.HATCH_GRAB_INSIDE),
				new FollowVisionTargetTheSecond(5.9),
				// new DrivePowerToVisionTarget(.3, 0.5),
				// yes.addParallel(new RunIntake(1, 0, 1));
				new DrivePowerAndIntake(0.4, -1, 0.8),
				// new DrivePower(-.4, 0.3));
				new StartEndCommand(() -> {DriveTrain.getInstance().arcadeDrive(-0.4, 0, false);}, DriveTrain.getInstance()::stop, DriveTrain.getInstance()));

	}
}
