package frc.robot.commands.auto.routines;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.team5940.pantry.exparimental.command.SendableCommandBase;
import org.team5940.pantry.exparimental.command.SequentialCommandGroup;

import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.drivetrain.PIDDriveDistance;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
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
public class PlaceCargoFrontL extends SequentialCommandGroup {
	// private SendableCommandBase mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	public PlaceCargoFrontL(char arg1, char arg2) {
		this();
	}

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine. 
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public PlaceCargoFrontL(/* char startPos, char side */) {
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
		// TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habL" + " to " + "cargoML"); //current trajectory from hashmap in Trajectories

		var traject = Trajectories.generateTrajectoryLowGear(Arrays.asList(
				// new Pose2d(LengthKt.getFeet(5.1), LengthKt.getFeet(17.684), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(9.5), LengthKt.getFeet(17.684), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(15.1), LengthKt.getFeet(14.434), Rotation2dKt.getDegree(0))), false);

		addCommands(new SetGearCommand(Gear.LOW));

		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(6), VelocityKt.getVelocity(LengthKt.getFeet(7)), false));
		addCommands(new PIDDriveDistance(LengthKt.getFeet(5), 6));

		// addSequential(new DelayCommand(TimeUnitsKt.getSecond(1)).getWrappedValue());
		// addSequential(new WaitCommand(0.7));

		// addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)); // move arm inside to prep state

		addCommands(parallel(
				new JankyGoToState(iPosition.HATCH_GRAB_INSIDE),
				new LimeLight.SetLEDs(LimeLight.LEDMode.kON)));

		//		addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE)); // move arm inside to prep state
		//		addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));

		addCommands(new LimeLight.setPipeline(PipelinePreset.k3dVision));

		addCommands(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		// addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		addCommands(new FollowVisionTargetTheSecond(5));

		// addSequential(new DriveDistanceTheThird(LengthKt.getFeet(0.4), false));

		// addSequential(new PrintCommand("GOT TO RUN INTAKE"));

		addCommands(new RunIntake(-1, 0, 1));

		addCommands(new PIDDriveDistance(-3, 12, /* timeout */ 0.5));

		// addSequential(new DriveDistanceTheThird(LengthKt.getFeet(2), true));
	}

	// id functions

	/**
	 * identification function
	 * @return
	 *  the mBigCommandGroup of the function
	 */
	public SendableCommandBase getBigCommandGroup() {
		return this;
	}

	//not id functions

}
