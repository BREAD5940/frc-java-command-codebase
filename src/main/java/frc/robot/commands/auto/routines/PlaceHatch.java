// package frc.robot.commands.auto.routines;

// import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
// import org.ghrobotics.lib.mathematics.units.LengthKt;

// import org.team5940.pantry.exparimental.command.PrintCommand;
// 
// import frc.robot.commands.subsystems.drivetrain.DrivePower;
// import frc.robot.commands.subsystems.drivetrain.SplineToVisionTarget;
// import frc.robot.commands.subsystems.superstructure.RunIntake;
// import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
// import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

// public class PlaceHatch extends SequentialCommandGroup {

// 	// Pose2d mMeasuredPose = new Pose2d();

// 	// public void setPose(Pose2d newPose) {
// 	// 	System.out.println("============================ Calleback called! ============================");
// 	// 	System.out.println("New pose: " + mMeasuredPose.getRotation().getDegree());
// 	// 	this.mMeasuredPose = newPose;
// 	// }

// 	public PlaceHatch() {
// 		this(new Pose2d(), false);
// 	}

// 	/**
// 	 * Place a hatch using a vision target tracker followed by a spline target. No Splines, yet. TODO splines for this boi
// 	 * 
// 	 * @author Matthew Morley
// 	 */
// 	public PlaceHatch(Pose2d goalPose, boolean setOdometry) {

// 		addSequential(new PrintCommand("Placing a hatch;;;...."));

// 		addParallel(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE.elevator.plus(iPosition.kOffsetFromL1toL2), iPosition.HATCH_SLAM_ROCKET_INSIDE.jointAngles));
// 		addSequential(new SplineToVisionTarget(/*this.getPoseStorage1(), */LengthKt.getInch(0), LengthKt.getInch(30), 6.5));
// 		addSequential(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE), 1);
// 		addSequential(new RunIntake(-1, 0, 1));

// 		addParallel(new RunIntake(-1, 0, 0.75));
// 		addSequential(new DrivePower(-0.2, 0.75));

// 		addSequential(new PrintCommand("We done bois, hatch placed...."));

// 		// addParallel(new SuperstructureGoToState(new ElevatorState(fieldPositions.hatchLowGoal.plus(LengthKt.getInch(3))), iPosition.HATCH_PITCHED_UP));

// 		// addSequential(new PrintCommand("driving at a power in reverse"));

// 		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(.5), true)); // TODO run the next spline, saves time, vs backing up
// 	}
// }
