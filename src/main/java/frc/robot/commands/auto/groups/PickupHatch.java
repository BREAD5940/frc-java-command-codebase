package frc.robot.commands.auto.groups;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.drivetrain.DrivePower;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PickupHatch extends CommandGroup {

	/**
	 * Pickup a hatch from the loading station using some jank open loop code.
	 * Vision code coming soon, I hope
	 * 
	 * @author Matthew Morley
	 */
	public PickupHatch() {

		addSequential(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));

		addSequential(new FollowVisionTargetTheSecond(6.5)); // in low res mode

		// Consumer<Pose2d> translationSetter = (Pose2d newP) -> {this.mMeasuredPose = newP;};

		// addSequential(new PrintCommand("pose: " + this.mMeasuredPose.getRotation().getDegree()));

		// addSequential(new SetPoseFromVisionTarget(this::setPose));

		// addSequential(new PrintCommand("pose: " + this.mMeasuredPose.getRotation().getDegree()));

		// next we drive forward a foot or two

		addParallel(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE));

		// addSequential(new DrivePower(0.4, 1));

		// addSequential(new PrintCommand("intaking...."));

		// addParallel(new RunIntake(-1, 1));

		addSequential(new PrintCommand("driving at a power"));

		addSequential(new DrivePower(0.3, 1.5));

		addParallel(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal.plus(LengthKt.getInch(3))), iPosition.HATCH_GRAB_INSIDE.getAngle()));

		addSequential(new PrintCommand("driving at a power in reverse"));

		addSequential(new DrivePower(-0.2, 0.75));

		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(.5), true)); // TODO run the next spline, saves time, vs backing up
	}
}
