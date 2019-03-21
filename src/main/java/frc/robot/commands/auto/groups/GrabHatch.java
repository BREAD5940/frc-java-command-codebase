package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.drivetrain.DrivePower;
import frc.robot.commands.subsystems.drivetrain.DrivePowerAndIntake;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.subsystems.Intake.HatchMechState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class GrabHatch extends CommandGroup {

	/**
	 * Pickup a hatch from the loading station using some jank open loop code.
	 * Vision code coming soon, I hope
	 * 
	 * @author Matthew Morley
	 */
	public GrabHatch() {

		this.addSequential(new SetHatchMech(HatchMechState.kClamped));
		// yes.addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		this.addSequential(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));
		this.addSequential(new FollowVisionTargetTheSecond(5.9));
		// new DrivePowerToVisionTarget(.3, 0.5),
		// yes.addParallel(new RunIntake(1, 0, 1));
		this.addSequential(new DrivePowerAndIntake(0.4, -1, 0.8));
		this.addSequential(new DrivePower(-.4, 0.3));

	}
}
