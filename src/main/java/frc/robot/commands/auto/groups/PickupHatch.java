package frc.robot.commands.auto.groups;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceTheSecond;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.lib.obj.factories.SequentialCommandFactory;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.PipelinePreset;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PickupHatch extends CommandGroup {

	/**
	 * Pickup a hatch from the loading station using some jank open loop code.
	 * Vision code coming soon, I hope
	 * 
	 * @author Matthew Morley
	 */
	public PickupHatch() {

		addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));

		addSequential(new LimeLight.setPipeline(PipelinePreset.k3dVision));

		addSequential(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));

		addSequential(new FollowVisionTargetTheSecond(8)); // in high res mode

		// ======================================================================

		addSequential(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE));

		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(1), false)); // TODO run the next spline, saves time, vs backing up




		// addSequential(SequentialCommandFactory.getSequentialCommands(Arrays.asList(
				// new WaitCommand(0.75),
				// new RunIntake(1, 0, 0.5))));

		addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(1), true)); // TODO run the next spline, saves time, vs backing up

		addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));

	}
}
