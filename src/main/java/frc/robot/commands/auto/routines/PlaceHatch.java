package frc.robot.commands.auto.routines;

import java.util.Optional;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.drivetrain.DrivePower;
import frc.robot.commands.subsystems.drivetrain.FollowVisonTargetTheSecond;
import frc.robot.commands.subsystems.drivetrain.SetPoseFromVisionTarget;
import frc.robot.commands.subsystems.drivetrain.VisionSplineTest;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.lib.obj.factories.VisionTargetFactory;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PlaceHatch extends CommandGroup {

	// Pose2d mMeasuredPose = new Pose2d();

	// public void setPose(Pose2d newPose) {
	// 	System.out.println("============================ Calleback called! ============================");
	// 	System.out.println("New pose: " + mMeasuredPose.getRotation().getDegree());
	// 	this.mMeasuredPose = newPose;
	// }

	public PlaceHatch() {
		this(new Pose2d(), false);
	}

	/**
	 * Place a hatch using a vision target tracker followed by a spline target. No Splines, yet. TODO splines for this boi
	 * 
	 * @author Matthew Morley
	 */
	public PlaceHatch(Pose2d goalPose, boolean setOdometry) {

		addSequential(new PrintCommand("Placing a hatch;;;...."));

		addSequential(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal), iPosition.HATCH));

		addSequential(new PrintCommand("=== running vision target follower ==="));

		addSequential(new FollowVisonTargetTheSecond(2.8));

		addSequential(new VisionSplineTest(20, 6.5)); // todo check numbers

		// addSequential(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchHighGoal), iPosition.HATCH));

		// Consumer<Pose2d> translationSetter = (Pose2d newP) -> {this.mMeasuredPose = newP;};

		// addSequential(new PrintCommand("pose: " + this.mMeasuredPose.getRotation().getDegree()));

		addSequential(new SetPoseFromVisionTarget(goalPose));

		// addSequential(new PrintCommand("pose: " + this.mMeasuredPose.getRotation().getDegree()));

		// next we drive forward a foot or two

		// addParallel(new SuperstructureGoToState(iPosition.HATCH));

		// addSequential(new PrintCommand("driving forward at a power"));


		addSequential(new PrintCommand("outtaking...."));

		addParallel(new RunIntake(-1, 0.75));
		addSequential(new DrivePower(0.15, 0.75));

		addParallel(new RunIntake(-1, 0.75));
		addSequential(new DrivePower(-0.2, 0.75));


		addSequential(new PrintCommand("We done bois, hatch placed...."));

		// addParallel(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal.plus(LengthKt.getInch(3))), iPosition.HATCH_PITCHED_UP));

		// addSequential(new PrintCommand("driving at a power in reverse"));


		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(.5), true)); // TODO run the next spline, saves time, vs backing up
	}
}
