package frc.robot.commands.auto.groups;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team5940.pantry.exparimental.command.PrintCommand;
import frc.robot.commands.auto.Trajectories;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import org.team5940.pantry.exparimental.command.SequentialCommandGroup;

public class visionTest extends SequentialCommandGroup {
	/**
	 * Nyoooooooooooooooooooooooooooooooooooooooooooom
	 * <P>
	 * Literally
	 * <P>
	 * Right now this goes from loading station to rocket using a Ramsete tracker and 
	 * then uses vision splines from there. This is for messing with pathing and vision.
	 * 
	 * @deprecated
	 * 
	 * @author Matthew Morley
	 * 
	 */
	@Deprecated
	public visionTest() {
		// addSequential(new VisionSplineTest(25, 4)); // todo check numbers

		// addSequential(new FollowVisonTargetTheSecond(2.9));

		// new Pose2dWithCurvature(pose, curvature)

		// go from loading statin to rocket

		List<Pose2d> waypoints = Arrays.asList(
				new Pose2d(LengthKt.getFeet(1.393), LengthKt.getFeet(2.147), Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(14.187), LengthKt.getFeet(3.45), Rotation2dKt.getDegree(-30)));

		TimedTrajectory<Pose2dWithCurvature> trajectory = Trajectories.generateTrajectory(waypoints, Trajectories.kLowGearConstraints, Trajectories.kDefaultStartVelocity, VelocityKt.getVelocity(LengthKt.getFeet(2)), VelocityKt.getVelocity(LengthKt.getFeet(5)), Trajectories.kDefaultAcceleration.div(2), false, true);

		addCommands(DriveTrain.getInstance().followTrajectoryWithGear(trajectory, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true));

		addCommands(new PrintCommand("==================================== moving on to next command ======================="));

		// addSequential(new SplineToVisionTarget(, 6.5)); // todo check numbers

	}
}
