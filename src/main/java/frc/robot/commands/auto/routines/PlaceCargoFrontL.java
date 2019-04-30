package frc.robot.commands.auto.routines;

import java.util.ArrayList;
import java.util.Arrays;

import frc.robot.commands.auto.PrettyAutoMotion;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.team5940.pantry.experimental.command.InstantCommand;

import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.drivetrain.PIDDriveDistance;
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
public class PlaceCargoFrontL extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<PrettyAutoMotion> motions = new ArrayList<PrettyAutoMotion>();

	public PlaceCargoFrontL(char arg1, char arg2) {
		this();
	}

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine.
	 * @author Matthew Morley
	 */
	public PlaceCargoFrontL(/* char startPos, char side */) {

		var traject = Trajectories.generateTrajectoryLowGear(Arrays.asList(
				// new Pose2d(LengthKt.getFeet(5.1), LengthKt.getFeet(17.684), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(9.5), LengthKt.getFeet(17.684), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(15.1), LengthKt.getFeet(14.434), Rotation2dKt.getDegree(0))), false);

		addCommands(

				// /*addSequential*/(new SetGearCommand(Gear.LOW)),
				new InstantCommand(DriveTrain.getInstance()::setLowGear),
				/*addSequential*/(new PIDDriveDistance(LengthKt.getFeet(5), 6)).alongWith(
						/*addParallel*/(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE))), // move arm inside to prep state
				/*addSequential*/(new LimeLight.SetLEDs(LimeLight.LEDMode.kON)),
				/*addSequential*/(new LimeLight.setPipeline(PipelinePreset.k3dVision)),
				/*addSequential*/(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)), //drive to goal
				/*addSequential*/(new FollowVisionTargetTheSecond(5)),
				/*addSequential*/(new RunIntake(-1, 0, 1)),
				/*addSequential*/(new PIDDriveDistance(-3, 12, /* timeout */ 0.5))

		);

	}

}
