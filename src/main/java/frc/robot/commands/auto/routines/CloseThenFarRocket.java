package frc.robot.commands.auto.routines;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceToVisionTarget;
import frc.robot.commands.subsystems.drivetrain.DrivePower;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class CloseThenFarRocket extends CommandGroup {
	/**
	 * Semi-auto routine for placing on the far rocet followed by the close one; 
	 */
	public CloseThenFarRocket(char side) {

		boolean isLeft = (side == 'L' || side == 'l');

		var fallOFfHab = Arrays.asList(
				new Pose2d(LengthKt.getFeet(5.175),
						LengthKt.getFeet(17.689),
						Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(8.501),
						LengthKt.getFeet(18.455),
						Rotation2dKt.getDegree(40))

		);

		var floorToRocketF = Arrays.asList(
				new Pose2d(LengthKt.getFeet(8.501),
						LengthKt.getFeet(18.455),
						Rotation2dKt.getDegree(40)),
				new Pose2d(LengthKt.getFeet(15.14),
						LengthKt.getFeet(24.122),
						Rotation2dKt.getDegree(32)));

		var rocketCToLoading = Arrays.asList(
				new Pose2d(LengthKt.getFeet(15.73),
						LengthKt.getFeet(24.372),
						Rotation2dKt.getDegree(32)),
				new Pose2d(LengthKt.getFeet(6.89),
						LengthKt.getFeet(22.323),
						Rotation2dKt.getDegree(-7.022)),
				new Pose2d(LengthKt.getFeet(4),
						LengthKt.getFeet(24.9),
						Rotation2dKt.getDegree(180)),
				new Pose2d(LengthKt.getFeet(5),
						LengthKt.getFeet(24.9),
						Rotation2dKt.getDegree(180)));

		var loadingToRocketF = Arrays.asList(
				new Pose2d(LengthKt.getFeet(1.465),
						LengthKt.getFeet(24.871),
						Rotation2dKt.getDegree(180)),
				new Pose2d(LengthKt.getFeet(19.871),
						LengthKt.getFeet(22.601),
						Rotation2dKt.getDegree(-154.855)),
				new Pose2d(LengthKt.getFeet(23.772),
						LengthKt.getFeet(23.551),
						Rotation2dKt.getDegree(148)));

		var rocketFtoLoading = Arrays.asList(
				new Pose2d(LengthKt.getFeet(22.438),
						LengthKt.getFeet(24.194),
						Rotation2dKt.getDegree(148)),
				new Pose2d(LengthKt.getFeet(17.237),
						LengthKt.getFeet(22.619),
						Rotation2dKt.getDegree(-26.747)),
				new Pose2d(LengthKt.getFeet(3.493),
						LengthKt.getFeet(24.763),
						Rotation2dKt.getDegree(0)));

		if (!isLeft) {
			fallOFfHab = Util.reflectTrajectory(fallOFfHab);
			floorToRocketF = Util.reflectTrajectory(floorToRocketF);
			rocketCToLoading = Util.reflectTrajectory(rocketCToLoading);
			loadingToRocketF = Util.reflectTrajectory(loadingToRocketF);
			rocketFtoLoading = Util.reflectTrajectory(rocketFtoLoading);
		}

		var t_fallOFfHab = Trajectories.generateTrajectory(fallOFfHab, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(5.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var t_floorToRocketF = Trajectories.generateTrajectory(floorToRocketF, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(5.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(3.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var t_rocketLFToLoading = Trajectories.generateTrajectory(rocketCToLoading, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var t_loadingToRocketF = Trajectories.generateTrajectory(loadingToRocketF, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(7.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var t_rocketFtoLoading = Trajectories.generateTrajectory(rocketFtoLoading, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(7.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_fallOFfHab, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));
		addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_floorToRocketF, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, false));
		addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(30), VelocityKt.getVelocity(LengthKt.getFeet(3))));
		addParallel(new RunIntake(-1, 0, 1));
		addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_rocketLFToLoading, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, false));
		addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(20), VelocityKt.getVelocity(LengthKt.getFeet(3))));
		addParallel(new RunIntake(1, 0, 1));
		addSequential(new DrivePower(0.3, 1));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_loadingToRocketF, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));
		addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(30), VelocityKt.getVelocity(LengthKt.getFeet(3))));
    addParallel(new RunIntake(-1, 0, 1));
    addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));
    addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_rocketFtoLoading, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));


	}
}
