package frc.robot.commands.auto.routines.yeOldeRoutines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.Logger;

// @SuppressWarnings("WeakerAccess")
public class Trajectories {

	public static TimedTrajectory<Pose2dWithCurvature> reverse3FeetLowGear;// = generateTrajectoryLowGear(Arrays.asList(new Pose2d(LengthKt.getFeet(0), LengthKt.getFeet(0), Rotation2dKt.getDegree(180)), new Pose2d(LengthKt.getFeet(3), LengthKt.getFeet(0), Rotation2dKt.getDegree(180))), true);
	public static HashMap<String, Pose2d> locations = new HashMap<String, Pose2d>();
	public static Velocity<Length> yeetSpeed = VelocityKt.getVelocity(LengthKt.getFeet(12.5)); //FIXME what is the speed for the spin?

	/**
	 * WARNING: do NOT call this more than once. it gets VERY sad
	 * TODO actually figure out why that breaks it
	 */
	private static void genLocs() {
		locations.put("habL", new Pose2d(LengthKt.getFeet(5.6), LengthKt.getFeet(17.684), Rotation2dKt.getDegree(180)));
		locations.put("habM", new Pose2d(LengthKt.getFeet(5.6), LengthKt.getFeet(13.379), Rotation2dKt.getDegree(180)));
		locations.put("habR", new Pose2d(LengthKt.getFeet(5.6), LengthKt.getFeet(9.508), Rotation2dKt.getDegree(180)));
		locations.put("loadingL", new Pose2d(LengthKt.getFeet(1.286), LengthKt.getFeet(25.021), Rotation2dKt.getDegree(180.0)));
		locations.put("loadingR", new Pose2d(LengthKt.getFeet(1.325), LengthKt.getFeet(2.336), Rotation2dKt.getDegree(180.0)));
		locations.put("cargoL1", new Pose2d(LengthKt.getFeet(21.565), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(-90d)));
		locations.put("cargoL2", new Pose2d(LengthKt.getFeet(23.532), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(-90d)));
		locations.put("cargoL3", new Pose2d(LengthKt.getFeet(25.277), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(-90d)));
		locations.put("cargoML", new Pose2d(LengthKt.getFeet(17.101), LengthKt.getFeet(14.338), Rotation2dKt.getDegree(180)));
		locations.put("cargoMR", new Pose2d(LengthKt.getFeet(17.066), LengthKt.getFeet(12.653), Rotation2dKt.getDegree(180)));
		locations.put("cargoR1", new Pose2d(LengthKt.getFeet(21.565), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(90d)));
		locations.put("cargoR2", new Pose2d(LengthKt.getFeet(23.532), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(90d)));
		locations.put("cargoR3", new Pose2d(LengthKt.getFeet(25.277), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(90d)));
		locations.put("depotL", new Pose2d(LengthKt.getFeet(5.203), LengthKt.getFeet(20.517), Rotation2dKt.getDegree(0)));
		locations.put("depotR", new Pose2d(LengthKt.getFeet(5.203), LengthKt.getFeet(6.107), Rotation2dKt.getDegree(0)));
		locations.put("yeetL", new Pose2d(LengthKt.getFeet(13.606), LengthKt.getFeet(21.315), Rotation2dKt.getDegree(145)));
		locations.put("yeetR", new Pose2d(LengthKt.getFeet(13.606), LengthKt.getFeet(5.685), Rotation2dKt.getDegree(-145)));
		locations.put("pyeetL", new Pose2d(LengthKt.getFeet(13.606), LengthKt.getFeet(21.315), Rotation2dKt.getDegree(325)));
		locations.put("pyeetR", new Pose2d(LengthKt.getFeet(13.606), LengthKt.getFeet(5.685), Rotation2dKt.getDegree(35)));

	}

	public static HashMap<String, TimedTrajectory<Pose2dWithCurvature>> generatedHGTrajectories = new HashMap<String, TimedTrajectory<Pose2dWithCurvature>>();
	public static HashMap<String, TimedTrajectory<Pose2dWithCurvature>> generatedLGTrajectories = new HashMap<String, TimedTrajectory<Pose2dWithCurvature>>();
	public static List<String> grabs = new ArrayList<String>(Arrays.asList("habR", "habM", "habL", "loadingL", "loadingR", "depotLF", "depotLB", "depotRF", "depotRB"));
	public static List<String> puts = new ArrayList<String>(Arrays.asList("cargoL1", "cargoL2", "cargoL3", "cargoML", "cargoMR", "cargoR1", "cargoR2", "cargoR3",
			"rocketL1", "rocketL2", "rocketL3", "rocketR1", "rocketR2", "rocketR3"));

	public static final Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
	public static final Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));

	public static final Velocity<Length> kDefaultVelocityLow = VelocityKt.getVelocity(LengthKt.getFeet(4));
	public static final Velocity<Length> kDefaultVelocityHigh = VelocityKt.getVelocity(LengthKt.getFeet(9));

	public static final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(8));

	public static final boolean kOptomizeSplines = true;

	private static final ArrayList<Pose2d> forward20ftSrc = new ArrayList<Pose2d>(Arrays.asList(
			new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(5), Rotation2dKt.getDegree(0)),
			new Pose2d(LengthKt.getFeet(35), LengthKt.getFeet(5), Rotation2dKt.getDegree(0))));
	public static TimedTrajectory<Pose2dWithCurvature> forward20Feet;

	public static List<TimingConstraint<Pose2dWithCurvature>> kLowGearConstraints = Arrays.asList(
			new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(7))),
			new DifferentialDriveDynamicsConstraint(Constants.INSTANCE.getKLowGearDifferentialDrive(), 12 /* volts */),
			// This limits our velocity while within the given Rectangle2d to 2 feet per second (read: the hab)
			new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(5.0))));

	private static List<TimingConstraint<Pose2dWithCurvature>> kHighGearConstraints = Arrays.asList(
			new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(7))),
			new DifferentialDriveDynamicsConstraint(Constants.INSTANCE.getKHighGearDifferentialDrive(), 12 /* volts */),
			// This limits our velocity while within the given Rectangle2d to 2 feet per second (read: the hab)
			new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(5.0))));

	public static void generateAllTrajectories() {
		generateAllTrajectories(true);
	}

	/**
	 * Generate all trajectories for HIGH GEAR
	 * @param isReal
	 */
	public static void generateAllTrajectories(boolean isReal) {
		/** High gear trajectory for going forward 20ft! */
		forward20Feet = generateTrajectoryHighGear(forward20ftSrc, false);
		reverse3FeetLowGear = generateTrajectoryLowGear(Arrays.asList(new Pose2d(LengthKt.getFeet(0), LengthKt.getFeet(0), Rotation2dKt.getDegree(180)), new Pose2d(LengthKt.getFeet(3), LengthKt.getFeet(0), Rotation2dKt.getDegree(180))), true);

		Logger.log("Generating ALL trajectories");
		genLocs();
		double startTime = 0;
		if (isReal)
			startTime = Timer.getFPGATimestamp();

		generatedHGTrajectories.put("habL to cargoML", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("habL"), locations.get("cargoML"))), true));
		generatedHGTrajectories.put("habM to cargoMR", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("habM"), locations.get("cargoMR"))), true));
		generatedHGTrajectories.put("habM to cargoML", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("habM"), locations.get("cargoML"))), true));
		generatedHGTrajectories.put("habR to cargoMR", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("habR"), locations.get("cargoMR"))), true));

		generatedHGTrajectories.put("cargoML to loadingL", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoML"), locations.get("loadingL"))), false));
		generatedHGTrajectories.put("cargoMR to loadingR", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoMR"), locations.get("loadingR"))), false));

		generatedHGTrajectories.put("cargoL1 to depotL", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoL1"), locations.get("depotL"))), true));
		generatedHGTrajectories.put("cargoR1 to depotR", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoR1"), locations.get("depotR"))), true));

		generatedHGTrajectories.put("depotL to cargoL1", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("depotL"), locations.get("cargoL1"))), false));
		generatedHGTrajectories.put("depotR to cargoR1", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("depotR"), locations.get("cargoR1"))), false));

		generatedHGTrajectories.put("loadingL to cargoL1", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(
				locations.get("loadingL"), locations.get("cargoL1").plus(
						new Pose2d(
								LengthKt.getFeet(0),
								LengthKt.getFeet(0),
								Rotation2dKt.getDegree(-90))))),
				true));

		generatedHGTrajectories.put("loadingR to cargoR1", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(
				locations.get("loadingR"), locations.get("cargoR1").plus(
						new Pose2d(
								LengthKt.getFeet(0),
								LengthKt.getFeet(0),
								Rotation2dKt.getDegree(90))))),
				true));

		generatedHGTrajectories.put("test", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("loadingL"),
				new Pose2d(LengthKt.getFeet(15.9), LengthKt.getFeet(22.011), Rotation2dKt.getDegree(165)),
				new Pose2d(LengthKt.getFeet(21.646), LengthKt.getFeet(19.223), Rotation2dKt.getDegree(-90)))),
				kHighGearConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocityHigh, kDefaultAcceleration, true, kOptomizeSplines));
		generatedHGTrajectories.put("test1", generateTrajectoryHighGear(Arrays.asList(new Pose2d(LengthKt.getFeet(21.646), LengthKt.getFeet(19.223), Rotation2dKt.getDegree(-90)),
				locations.get("cargoL1")), false));

		generatedLGTrajectories.put("habL to cargoML", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("habL"), locations.get("cargoML"))), true));
		generatedLGTrajectories.put("habM to cargoMR", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("habM"), locations.get("cargoMR"))), true));
		generatedLGTrajectories.put("habM to cargoML", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("habM"), locations.get("cargoML"))), true));
		generatedLGTrajectories.put("habR to cargoMR", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("habR"), locations.get("cargoMR"))), true));

		generatedLGTrajectories.put("cargoML to loadingL", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoML"), locations.get("loadingL"))), false));
		generatedLGTrajectories.put("cargoMR to loadingR", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(
				new Pose2d(locations.get("cargoMR").getTranslation(), Rotation2dKt.getDegree(180)),

				new Pose2d(LengthKt.getFeet(9.6), LengthKt.getFeet(6.7), Rotation2dKt.getDegree(50)),

				locations.get("loadingR").plus(
						new Pose2d(LengthKt.getFeet(3), LengthKt.getFeet(0), Rotation2dKt.getDegree(0))))),
				false));

		generatedLGTrajectories.put("cargoL1 to depotL", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoL1"), locations.get("depotL"))), true));
		generatedLGTrajectories.put("cargoR1 to depotR", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoR1"), locations.get("depotR"))), true));

		generatedLGTrajectories.put("depotL to cargoL1", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("depotL"), locations.get("cargoL1"))), false));
		generatedLGTrajectories.put("depotR to cargoR1", generateTrajectoryLowGear(new ArrayList<Pose2d>(Arrays.asList(locations.get("depotR"), locations.get("cargoR1"))), false));

		generatedLGTrajectories.put(
				"loadingR to rocketRF",
				generateTrajectoryLowGear(
						// Arrays.asList(
						// 		locations.get("loadingR").plus(new Pose2d(
						// 			new Translation2d(), Rotation2dKt.getDegree(180))),
						// 		new Pose2d(
						// 				LengthKt.getFeet(19.375),
						// 				LengthKt.getFeet(4.634),
						// 				Rotation2dKt.getDegree(170)),
						// 		new Pose2d(
						// 				LengthKt.getFeet(23.841),
						// 				LengthKt.getFeet(3.42),
						// 				Rotation2dKt.getDegree(-150))),
						// true));

						Arrays.asList(
								new Pose2d(
										LengthKt.getFeet(1.8),
										LengthKt.getFeet(2.2),
										Rotation2dKt.getDegree(180)),
								new Pose2d(
										LengthKt.getFeet(19.5),
										LengthKt.getFeet(4.5),
										Rotation2dKt.getDegree(180)),

								new Pose2d(
										LengthKt.getFeet(23.92),
										LengthKt.getFeet(3.5),
										Rotation2dKt.getDegree(-147))

						),
						false));

		// Trajectories to the rocket from (REVERSED) on habL. format is rocket[L/R for left/right][C/M/F for close/middle/far]. These are offset to allow for a vision target to yeet into it
		generatedLGTrajectories.put("habL to rocketLF", generateTrajectoryLowGear(
				Arrays.asList(
						locations.get("habL"),
						new Pose2d(
								LengthKt.getFeet(19.445),
								LengthKt.getFeet(22.808),
								Rotation2dKt.getDegree(-160)),
						new Pose2d(
								LengthKt.getFeet(23.801),
								LengthKt.getFeet(23.509),
								Rotation2dKt.getDegree(150))),
				true));

		// Trajectories to the rocket from (REVERSED) on habL. format is rocket[L/R for left/right][C/M/F for close/middle/far]. These are offset to allow for a vision target to yeet into it
		generatedLGTrajectories.put("habR to rocketRF", generateTrajectoryLowGear(
				Arrays.asList(
						locations.get("habR"),
						new Pose2d(
								LengthKt.getFeet(20.33),
								LengthKt.getFeet(5.2),
								Rotation2dKt.getDegree(140)),
						new Pose2d(
								LengthKt.getFeet(25.001),
								LengthKt.getFeet(4.1),
								Rotation2dKt.getDegree(-150))),
				true));

		// generatedHGTrajectories.put("habR to rocketRF", generateTrajectoryHighGear(
		// 		Arrays.asList(
		// 				locations.get("habR"),
		// 				new Pose2d(
		// 						LengthKt.getFeet(19.8),
		// 						LengthKt.getFeet(5.1),
		// 						Rotation2dKt.getDegree(140)),
		// 				new Pose2d(
		// 						LengthKt.getFeet(23.801),
		// 						LengthKt.getFeet(3.5),
		// 						Rotation2dKt.getDegree(-150))),
		// 		true));

		// Trajectories to the rocket. format is rocket[L/R for left/right][C/M/F for close/middle/far]. This one is backed up 3 feet already!!!!!!!!!!!!!!!!!!!!!!!!!!
		generatedLGTrajectories.put("rocketLF to loadingL", generateTrajectoryLowGear(
				Arrays.asList(
						new Pose2d(
								LengthKt.getFeet(23.92),
								LengthKt.getFeet(23.28),
								Rotation2dKt.getDegree(-33)),
						new Pose2d(
								LengthKt.getFeet(19.5),
								LengthKt.getFeet(23.065),
								Rotation2dKt.getDegree(0)),
						locations.get("loadingL").plus(
								new Pose2d(LengthKt.getFeet(3), LengthKt.getFeet(0), Rotation2dKt.getDegree(0)) // offset by 3 feet to allow the vision spline to kick in a bit
						)),
				true));

		// Trajectories to the rocket. format is rocket[L/R for left/right][C/M/F for close/middle/far]. This one is backed up 3 feet already!!!!!!!!!!!!!!!!!!!!!!!!!!
		generatedLGTrajectories.put("rocketRF to loadingR", generateTrajectoryLowGear(
				Arrays.asList(
						new Pose2d(
								LengthKt.getFeet(23.92),
								LengthKt.getFeet(27 - 23.28),
								Rotation2dKt.getDegree(-147)),
						new Pose2d(
								LengthKt.getFeet(19.5),
								LengthKt.getFeet(4.5),
								Rotation2dKt.getDegree(180)),
						new Pose2d(
								LengthKt.getFeet(4.3),
								LengthKt.getFeet(2.3),
								Rotation2dKt.getDegree(180))),
				false));

		// path from loading station to rcket
		generatedLGTrajectories.put("loadingL to rocketLC", generateTrajectoryLowGear(
				Arrays.asList(
						new Pose2d(
								LengthKt.getFeet(1.447),
								LengthKt.getFeet(24.894),
								Rotation2dKt.getDegree(180)),
						new Pose2d(
								LengthKt.getFeet(13.013),
								LengthKt.getFeet(23.055),
								Rotation2dKt.getDegree(30))),
				true));

		generatedLGTrajectories.put("loadingL to cargoL1", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(
				locations.get("loadingL"), locations.get("cargoL1").plus(
						new Pose2d(
								LengthKt.getFeet(0),
								LengthKt.getFeet(0),
								Rotation2dKt.getDegree(-90))))),
				true));

		generatedLGTrajectories.put("loadingR to cargoR1", generateTrajectoryHighGear(new ArrayList<Pose2d>(Arrays.asList(
				locations.get("loadingR"), locations.get("cargoR1").plus(
						new Pose2d(
								LengthKt.getFeet(0),
								LengthKt.getFeet(0),
								Rotation2dKt.getDegree(90))))),
				true));

		generatedLGTrajectories.put("test", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("loadingL"),
				new Pose2d(LengthKt.getFeet(15.9), LengthKt.getFeet(22.011), Rotation2dKt.getDegree(165)),
				new Pose2d(LengthKt.getFeet(21.646), LengthKt.getFeet(19.223), Rotation2dKt.getDegree(-90)))),
				kLowGearConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocityLow, kDefaultAcceleration, true, kOptomizeSplines));
		generatedLGTrajectories.put("test1", generateTrajectoryLowGear(Arrays.asList(new Pose2d(LengthKt.getFeet(21.646), LengthKt.getFeet(19.223), Rotation2dKt.getDegree(-90)),
				locations.get("cargoL1")), false));

		System.out.println("Out of first round of generation");
		double now = 0;
		if (isReal) {
			now = Timer.getFPGATimestamp();
			Logger.log("Trajectories generated in " + (now - startTime) + " seconds!");
		}
	}

	/**
	 * Generate a trajectory from a list of waypoints in high gear
	 * @param waypoints to follow
	 * @param reversed if the path is reversed
	 * @return
	 */
	public static TimedTrajectory<Pose2dWithCurvature> generateTrajectoryHighGear(List<Pose2d> waypoints, boolean reversed) {
		return generateTrajectory(waypoints, kHighGearConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocityHigh, kDefaultAcceleration, reversed, kOptomizeSplines);
	}

	/**
	* Generate a trajectory from a list of waypoints in low gear
	* @param waypoints to follow
	* @param reversed if the path is reversed
	* @return
	*/
	public static TimedTrajectory<Pose2dWithCurvature> generateTrajectoryLowGear(List<Pose2d> waypoints, boolean reversed) {
		return generateTrajectory(waypoints, kLowGearConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocityLow, kDefaultAcceleration, reversed, kOptomizeSplines);
	}

	/**
	 * Generate a trajectory from scratch. Given a list of waypoints, constriants, velocities and accelerations, a trajectory will
	 * be generated using the default trajectory generator.
	 * @param waypoints to follow
	 * @param constraints_ to respect (slow zone, acceleration, etc)
	 * @param startVelocity to start at
	 * @param endVelocity to end at
	 * @param maxVelocity that will never be exceded
	 * @param maxAcceleration for the trajectory
	 * @param reversed for if the path should be reversed (flipped)
	 */
	public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
			List<? extends TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed, boolean optomizeSplines) {
		return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(
				waypoints,
				constraints_,
				startVelocity,
				endVelocity,
				maxVelocity,
				maxAcceleration,
				reversed, optomizeSplines);
	}

	public static TimedTrajectory<Pose2dWithCurvature> reflect(TimedTrajectory<Pose2dWithCurvature> unReflected) {
		List<Pose2d> newWaypoints = new ArrayList<>();
		Velocity<Length> maxVelocity = VelocityKt.getVelocity(LengthKt.getInch(0));
		Acceleration<Length> maxAccel = AccelerationKt.getAcceleration(LengthKt.getInch(0));
		for (TimedEntry<Pose2dWithCurvature> point : unReflected.getPoints()) {
			maxVelocity = (point.getVelocity().getValue() > maxVelocity.getValue()) ? point.getVelocity() : maxVelocity;
			maxAccel = (point.getAcceleration().getValue() > maxAccel.getValue()) ? point.getAcceleration() : maxAccel;
			double centerOffset = (13.5 - point.getState().getPose().getTranslation().getY().getFeet()) * -1;
			newWaypoints.add(new Pose2d(
					new Translation2d(LengthKt.getFeet(centerOffset + 13.5),
							point.getState().getPose().getTranslation().getY()),
					new Rotation2d(point.getState().getPose().getRotation().getRadian() * -1)));
		}

		return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(
				newWaypoints,
				kLowGearConstraints, //FIXME find gear of other traject
				unReflected.getFirstState().getVelocity(), //FIXME i think this does what I want it to
				unReflected.getLastState().getVelocity(),
				maxVelocity,
				maxAccel,
				unReflected.getReversed(),
				unReflected.getReversed());
	}

}
