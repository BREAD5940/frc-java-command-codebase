package frc.robot.commands.auto.routines;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.motion.Util;

public class CloseThenFarRocket extends CommandGroup {
	/**
	 * Semi-auto routine for placing on the far rocet followed by the close one; 
	 */
	public CloseThenFarRocket(char side) {

		boolean isLeft = (side == 'L' || side == 'l');

		var fallOFfHabL = Arrays.asList(
				new Pose2d(LengthKt.getFeet(5.175),
						LengthKt.getFeet(17.689),
						Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(8.501),
						LengthKt.getFeet(18.455),
						Rotation2dKt.getDegree(40))

		);

		var floorToRocketLF = Arrays.asList(
				new Pose2d(LengthKt.getFeet(8.501),
						LengthKt.getFeet(18.455),
						Rotation2dKt.getDegree(40)),
				new Pose2d(LengthKt.getFeet(15.14),
						LengthKt.getFeet(24.122),
						Rotation2dKt.getDegree(32)));

		var rocketLFToLoadingL = Arrays.asList(
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

		if (!isLeft) {
			fallOFfHabL = Util.reflectTrajectory(fallOFfHabL);
			floorToRocketLF = Util.reflectTrajectory(floorToRocketLF);
		}

		var t_fallOFfHabL = Trajectories.generateTrajectory(fallOFfHabL, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(5.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(7.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var t_floorToRocketLF = Trajectories.generateTrajectory(floorToRocketLF, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(5.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(7.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

	}
}
