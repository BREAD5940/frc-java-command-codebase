package frc.robot.commands.auto.routines;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceToVisionTarget;
import frc.robot.commands.subsystems.drivetrain.TurnToFaceVisionTarget;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.lib.ParallelRaceGroup;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;
import frc.robot.xboxmap;

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
				new Pose2d(LengthKt.getFeet(8),
						LengthKt.getFeet(18.5),
						Rotation2dKt.getDegree(40))

		);

		var floorToRocketC = Arrays.asList(
				new Pose2d(LengthKt.getFeet(8.0),
						LengthKt.getFeet(18.5),
						Rotation2dKt.getDegree(40)),
				new Pose2d(LengthKt.getFeet(14.14),
						LengthKt.getFeet(24.5),
						Rotation2dKt.getDegree(27)));

		var rocketCToLoading = Arrays.asList(
				new Pose2d(LengthKt.getFeet(15.7),
						LengthKt.getFeet(24.37),
						Rotation2dKt.getDegree(32)),
				new Pose2d(LengthKt.getFeet(5.572),
						LengthKt.getFeet(23.017),
						Rotation2dKt.getDegree(-10)),
				new Pose2d(LengthKt.getFeet(5),
						LengthKt.getFeet(24.918),
						Rotation2dKt.getDegree(180)),
				new Pose2d(LengthKt.getFeet(6.5),
						LengthKt.getFeet(24.882),
						Rotation2dKt.getDegree(180)));

		// var loadingToRocketF = Arrays.asList(
		// 		new Pose2d(LengthKt.getFeet(1.465),
		// 				LengthKt.getFeet(24.871),
		// 				Rotation2dKt.getDegree(180)),
		// 		new Pose2d(LengthKt.getFeet(19.871),
		// 				LengthKt.getFeet(22.601),
		// 				Rotation2dKt.getDegree(-154.855)),
		// 		new Pose2d(LengthKt.getFeet(23.772),
		// 				LengthKt.getFeet(23.551),
		// 				Rotation2dKt.getDegree(148)));

		// var rocketFtoLoading = Arrays.asList(
		// 		new Pose2d(LengthKt.getFeet(22.438),
		// 				LengthKt.getFeet(24.194),
		// 				Rotation2dKt.getDegree(148)),
		// 		new Pose2d(LengthKt.getFeet(21.338),
		// 				LengthKt.getFeet(22.673),
		// 				Rotation2dKt.getDegree(0)),
		// 		new Pose2d(LengthKt.getFeet(6.023),
		// 				LengthKt.getFeet(22.892),
		// 				Rotation2dKt.getDegree(-4)),
		// 		new Pose2d(LengthKt.getFeet(4.5),
		// 				LengthKt.getFeet(24.8),
		// 				Rotation2dKt.getDegree(180)));

		if (!isLeft) {
			fallOFfHab = Util.reflectTrajectory(fallOFfHab);
			floorToRocketC = Util.reflectTrajectory(floorToRocketC);
			// rocketCToLoading = Util.reflectTrajectory(rocketCToLoading);
			// loadingToRocketF = Util.reflectTrajectory(loadingToRocketF);
			// rocketFtoLoading = Util.reflectTrajectory(rocketFtoLoading);
		}

		var t_fallOFfHab = Trajectories.generateTrajectory(fallOFfHab, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(7.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var t_floorToRocketC = Trajectories.generateTrajectory(floorToRocketC, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(7.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.5)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var rocketCtoTurn = Arrays.asList(
				new Pose2d(LengthKt.getFeet(15.435),
						LengthKt.getFeet(24.426),
						Rotation2dKt.getDegree(30)),
				new Pose2d(LengthKt.getFeet(11.136),
						LengthKt.getFeet(20),
						Rotation2dKt.getDegree(90)));

		if (!isLeft)
			rocketCtoTurn = Util.reflectTrajectory(rocketCtoTurn);

		var p_rocketCtoTurn = Trajectories.generateTrajectory(
				rocketCtoTurn,
				Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				true,
				true);

		var turnToLoading = Arrays.asList(
				new Pose2d(LengthKt.getFeet(11.136),
						LengthKt.getFeet(20),
						Rotation2dKt.getDegree(90)),
				new Pose2d(LengthKt.getFeet(5),
						LengthKt.getFeet(24.782),
						Rotation2dKt.getDegree(180)));

		if (!isLeft)
			turnToLoading = Util.reflectTrajectory(turnToLoading);

		var p_turnToLoading = Trajectories.generateTrajectory(
				turnToLoading,
				Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var loadingToRocketF = Arrays.asList(
				new Pose2d(LengthKt.getFeet(1.811),
						LengthKt.getFeet(24.925),
						Rotation2dKt.getDegree(180)),
				new Pose2d(LengthKt.getFeet(15.141),
						LengthKt.getFeet(23.855),
						Rotation2dKt.getDegree(168.462)),
				new Pose2d(LengthKt.getFeet(19.456),
						LengthKt.getFeet(22.946),
						Rotation2dKt.getDegree(-174.87)),
				new Pose2d(LengthKt.getFeet(23.685),
						LengthKt.getFeet(23.748),
						Rotation2dKt.getDegree(147.752)));

		if (!isLeft)
			loadingToRocketF = Util.reflectTrajectory(loadingToRocketF);

		var p_loadingToRocketF = Trajectories.generateTrajectory(
				turnToLoading,
				Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				true,
				true);

		var rocketFToLoadingIsh = Arrays.asList(
				new Pose2d(LengthKt.getFeet(22.351),
						LengthKt.getFeet(24.46),
						Rotation2dKt.getDegree(148)),
				new Pose2d(LengthKt.getFeet(5.78),
						LengthKt.getFeet(22.678),
						Rotation2dKt.getDegree(-21.852)),
				new Pose2d(LengthKt.getFeet(9.074),
						LengthKt.getFeet(24.835),
						Rotation2dKt.getDegree(180)));

		if (!isLeft)
			rocketFToLoadingIsh = Util.reflectTrajectory(rocketFToLoadingIsh);

		var p_rocketFToLoadingIsh = Trajectories.generateTrajectory(
				rocketFToLoadingIsh,
				Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				true,
				true);

		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_fallOFfHab, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));
		addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_floorToRocketC, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, false));

		addSequential(new TurnToFaceVisionTarget());

		addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(35), VelocityKt.getVelocity(LengthKt.getFeet(2))));

		addSequential(new TeleopCommands());

		addSequential(new ParallelRaceGroup(() -> (Robot.m_oi.getPrimary().getRawButton(xboxmap.Buttons.A_BUTTON)), new TeleopCommands()));

		addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));

		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(p_rocketCtoTurn, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));

		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(p_turnToLoading, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, false));

		// addSequential(new ParallelRaceGroup(() -> (Robot.m_oi.getPrimary().getRawButton(xboxmap.Buttons.A_BUTTON)) , new TeleopCommands()));

		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(p_loadingToRocketF, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));

		// addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));

		// addSequential(new ParallelRaceGroup(() -> (Robot.m_oi.getPrimary().getRawButton(xboxmap.Buttons.A_BUTTON)) , new TeleopCommands()));

		// addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(20), VelocityKt.getVelocity(LengthKt.getFeet(3))));

		// addSequential(new GrabHatch());

		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_loadingToRocketF, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));
		// addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));

		// addSequential(new PlaceHatch());

		// addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));
		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_rocketFtoLoading, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));

	}
}
