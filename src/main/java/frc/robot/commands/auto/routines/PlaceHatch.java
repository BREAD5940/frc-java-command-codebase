package frc.robot.commands.auto.routines;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.drivetrain.DrivePower;
import frc.robot.commands.subsystems.drivetrain.FollowVisonTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PlaceHatch extends CommandGroup {

	Pose2d mMeasuredPose = new Pose2d();

	public void setPose(Pose2d newPose) {
		System.out.println("============================ Calleback called! ============================");
		System.out.println("New pose: " + mMeasuredPose.getRotation().getDegree());
		this.mMeasuredPose = newPose;
	}

	/**
	 * Pickup a hatch from the loading station
	 */
	public PlaceHatch() {

		addSequential(new PrintCommand("Placing a hatch;;;...."));

		addSequential(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal), iPosition.HATCH));

		addSequential(new PrintCommand("=== running vision target follower ==="));

		addSequential(new FollowVisonTargetTheSecond(5));

		// Consumer<Pose2d> translationSetter = (Pose2d newP) -> {this.mMeasuredPose = newP;};

		// addSequential(new PrintCommand("pose: " + this.mMeasuredPose.getRotation().getDegree()));

		// addSequential(new SetPoseFromVisionTarget(this::setPose));

		// addSequential(new PrintCommand("pose: " + this.mMeasuredPose.getRotation().getDegree()));

		// next we drive forward a foot or two

		// addParallel(new SuperstructureGoToState(iPosition.HATCH));

		// addSequential(new PrintCommand("driving forward at a power"));

		addSequential(new DrivePower(0.15, 0.75));

		addSequential(new PrintCommand("outtaking...."));

		addSequential(new RunIntake(-1, 1));


		addSequential(new PrintCommand("We done bois, hatch placed...."));

		// addParallel(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal.plus(LengthKt.getInch(3))), iPosition.HATCH_PITCHED_UP));

		// addSequential(new PrintCommand("driving at a power in reverse"));

		addSequential(new DrivePower(-0.2, 1));

		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(.5), true)); // TODO run the next spline, saves time, vs backing up
	}
}
