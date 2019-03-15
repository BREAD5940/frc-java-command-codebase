package frc.robot.commands.auto.routines;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.commands.DelayCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceTheSecond;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceTheThird;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.drivetrain.PIDDriveDistance;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
import frc.robot.commands.subsystems.superstructure.ArmMove;
import frc.robot.commands.subsystems.superstructure.ElevatorMove;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.lib.obj.factories.SequentialCommandFactory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.PipelinePreset;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * 2-hatch 1-cargo auto
 */
public class PlaceCargoFrontL extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
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
				new Pose2d(LengthKt.getFeet(15.1), LengthKt.getFeet(14.434), Rotation2dKt.getDegree(0))
			), false);

		addSequential(new SetGearCommand(Gear.LOW));

		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(6), VelocityKt.getVelocity(LengthKt.getFeet(7)), false));
		addSequential(new PIDDriveDistance(LengthKt.getFeet(5), 6));

		// addSequential(new DelayCommand(TimeUnitsKt.getSecond(1)).getWrappedValue());
		// addSequential(new WaitCommand(0.7));


		// addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)); // move arm inside to prep state
		addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE)); // move arm inside to prep state


		addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));
		addSequential(new LimeLight.setPipeline(PipelinePreset.k3dVision));

		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		// addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		addSequential(new FollowVisionTargetTheSecond(5));



		// addSequential(new DriveDistanceTheThird(LengthKt.getFeet(0.4), false));

		// addSequential(new PrintCommand("GOT TO RUN INTAKE"));

		addSequential(new RunIntake(-1, 0, 1));


		addSequential(new PIDDriveDistance(-3, 12));

		// addSequential(new DriveDistanceTheThird(LengthKt.getFeet(2), true));
	}

	// id functions

	/**
	 * identification function
	 * @return
	 *  the mBigCommandGroup of the function
	 */
	public AutoCommandGroup getBigCommandGroup() {
		return this;
	}

	//not id functions

}
