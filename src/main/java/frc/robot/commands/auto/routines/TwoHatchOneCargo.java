package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.team5940.pantry.experimental.command.PrintCommand;
import org.team5940.pantry.experimental.command.SequentialCommandGroup;
import org.team5940.pantry.experimental.command.WaitCommand;

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
	// private AutoCommandGroup mBigCommandGroup;
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

		/* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habR" + " to " + "rocketRF"); //current trajectory from hashmap in Trajectories
		var rocketToLoading = Trajectories.generatedLGTrajectories.get("rocketRF to loadingR");
		var loadingToRocketFar = Trajectories.generatedLGTrajectories.get("loadingR to rocketRF");

		addCommands(

				/*addSequential*/(new LimeLight.SetLEDs(LimeLight.LEDMode.kON)),
				/*addSequential*/(new LimeLight.setPipeline(PipelinePreset.k3dVision)),
				/*addSequential*/(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)).alongWith(
						/*addParallel*/(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)) // move arm inside to prep state
				), //drive to goal

				/*addSequential*/(new FollowVisionTargetTheSecond(3.8)),

				/*addSequential*/(new RunIntake(-1, 0, 1.5)),

				/*addParallel*/(new DriveDistanceTheThird(LengthKt.getFeet(3), true)),

				// /*addSequential*/(SequentialCommandFactory.getSequentialCommands(
				// Arrays.asList(
				new ElevatorMove(iPosition.HATCH_GRAB_INSIDE.getElevator()),
				new JankyGoToState(iPosition.HATCH_GRAB_INSIDE),

				/*addSequential*/(new PrintCommand("GOT TO next spline")),

				// spline over to the rocket
				/*addSequential*/(DriveTrain.getInstance().followTrajectoryWithGear(rocketToLoading, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)), //drive to goal

				/*addSequential*/(new FollowVisionTargetTheSecond(4.5)),

				/*addSequential*/(new RunIntake(1, 0, 1)),

				// /*addParallel*/(SequentialCommandFactory.getSequentialCommands(
				// Arrays.asList(

				/*addSequential*/(DriveTrain.getInstance().followTrajectoryWithGear(loadingToRocketFar, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)).alongWith(
						(new WaitCommand(2)).andThen(
								new ArmMove(iPosition.HATCH)).andThen(
										new ElevatorMove(fieldPositions.hatchMiddleGoal))

				), //drive to goal
				/*addSequential*/(new FollowVisionTargetTheSecond(3.8)),
				/*addSequential*/(new RunIntake(-1, 0, 1))

		// // // /*addSequential*/(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));
		);
	}

}
