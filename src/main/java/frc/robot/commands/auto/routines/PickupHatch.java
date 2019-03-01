package frc.robot.commands.auto.routines;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.drivetrain.DrivePower;
import frc.robot.commands.subsystems.drivetrain.FollowVisonTargetTheSecond;
import frc.robot.commands.subsystems.drivetrain.SetPoseFromVisionTarget;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PickupHatch extends CommandGroup {

	/**
	 * Pickup a hatch from the loading station
	 */
	public PickupHatch() {

		addSequential(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));

		addSequential(new FollowVisonTargetTheSecond(7.5));

		// Consumer<Pose2d> translationSetter = (Pose2d newP) -> {this.mMeasuredPose = newP;};

		// addSequential(new PrintCommand("pose: " + this.mMeasuredPose.getRotation().getDegree()));

		// addSequential(new SetPoseFromVisionTarget(this::setPose));

		// addSequential(new PrintCommand("pose: " + this.mMeasuredPose.getRotation().getDegree()));

		// next we drive forward a foot or two

		// addParallel(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE));

		addSequential(new PrintCommand("intaking...."));

		// addParallel(new RunIntake(-1, 1));

		addSequential(new PrintCommand("driving at a power"));

		addSequential(new DrivePower(0.135, 0.9));

		addParallel(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal.plus(LengthKt.getInch(3))), iPosition.HATCH_PITCHED_UP));

		addSequential(new PrintCommand("driving at a power in reverse"));

		addSequential(new DrivePower(-0.2, 0.75));

		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(.5), true)); // TODO run the next spline, saves time, vs backing up
	}
}
