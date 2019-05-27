package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.team5940.pantry.exparimental.command.CommandGroupBase;
import org.team5940.pantry.exparimental.command.PrintCommand;
import org.team5940.pantry.exparimental.command.SequentialCommandGroup;
import org.team5940.pantry.exparimental.command.WaitCommand;

import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceTheThird;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.ArmMove;
import frc.robot.commands.subsystems.superstructure.ElevatorMove;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.PipelinePreset;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * 2-hatch 1-cargo auto
 */
public class TwoHatchOneCargo extends SequentialCommandGroup {
	// private SendableCommandBase mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	public TwoHatchOneCargo(char arg1, char arg2) {
		this();
	}

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine. 
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public TwoHatchOneCargo(/*char startPos, char side*/) {
		// HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		// String cStart = "hab" + startPos;

		// addSequential(new InstantRunnable(() -> {
		// 	SuperStructure.getElevator().getMaster().configPeakOutputForward(0);
		// 	SuperStructure.getElevator().getMaster().configPeakOutputReverse(0);

		// 	SuperStructure.getInstance().getWrist().getMaster().configPeakOutputForward(0);
		// 	SuperStructure.getInstance().getWrist().getMaster().configPeakOutputReverse(0);

		// 	SuperStructure.getInstance().getElbow().getMaster().configPeakOutputForward(0);
		// 	SuperStructure.getInstance().getElbow().getMaster().configPeakOutputReverse(0);
		// }, true));

		// boolean doIntake = false;
		// boolean doVision = false;

		/* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habR" + " to " + "rocketRF"); //current trajectory from hashmap in Trajectories

		//		addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)); // move arm inside to prep state

		addCommands(parallel(
				new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH),
				new LimeLight.SetLEDs(LimeLight.LEDMode.kON),
				new LimeLight.setPipeline(PipelinePreset.k3dVision),
				DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)));

		//		addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));
		//		addSequential(new LimeLight.setPipeline(PipelinePreset.k3dVision));
		//		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		// addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		addCommands(new FollowVisionTargetTheSecond(3.8));

		// addSequential(new DriveDistanceTheThird(LengthKt.getFeet(0.4), false));

		// addSequential(new PrintCommand("GOT TO RUN INTAKE"));

		addCommands(new RunIntake(-1, 0, 1.5));

		// addSequential(new PrintCommand("GOT TO BACKING UP"));

		// back up 3 feet
		// addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP)); // TODO fix this broken logic!

		addCommands(parallel(
				new DriveDistanceTheThird(LengthKt.getFeet(3), true),
				sequence(
						new ElevatorMove(iPosition.HATCH_GRAB_INSIDE.getElevator()),
						new JankyGoToState(iPosition.HATCH_GRAB_INSIDE))));

		//		addParallel(new DriveDistanceTheThird(LengthKt.getFeet(3), true));

		//		addSequential(SequentialCommandFactory.getSequentialCommands(
		//				Arrays.asList(
		//						new ElevatorMove(iPosition.HATCH_GRAB_INSIDE.getElevator()),
		//						new JankyGoToState(iPosition.HATCH_GRAB_INSIDE))));

		addCommands(new PrintCommand("GOT TO next spline"));

		// // spline over to the rocket
		var rocketToLoading = Trajectories.generatedLGTrajectories.get("rocketRF to loadingR");
		addCommands(DriveTrain.getInstance().followTrajectoryWithGear(rocketToLoading, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); //drive to goal

		// // // addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));

		addCommands(new FollowVisionTargetTheSecond(4.5));

		addCommands(new RunIntake(1, 0, 1));

		// addSequential(new DriveDistanceTheThird(LengthKt.getFeet(1), false));

		var loadingToRocketFar = Trajectories.generatedLGTrajectories.get("loadingR to rocketRF");

		//		addParallel(SequentialCommandFactory.getSequentialCommands(
		//				Arrays.asList(
		//						new WaitCommand("Wait for clearance", 2),
		//						new ArmMove(iPosition.HATCH),
		//						new ElevatorMove(fieldPositions.hatchMiddleGoal))));

		// // // addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));
		// addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)); // move arm inside to prep state
		//		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(loadingToRocketFar, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); //drive to goal

		addCommands(parallel(
				sequence(
						new WaitCommand(2),
						new ArmMove(iPosition.HATCH),
						new ElevatorMove(fieldPositions.hatchMiddleGoal))),
				DriveTrain.getInstance().followTrajectoryWithGear(loadingToRocketFar, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false));

		addCommands(new FollowVisionTargetTheSecond(3.8));
		addCommands(new RunIntake(-1, 0, 1));

		// // // addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));

	}

	// id functions

	/**
	 * identification function
	 * @return
	 *  the mBigCommandGroup of the function
	 */
	public CommandGroupBase getBigCommandGroup() {
		return this;
	}

	//not id functions

}
