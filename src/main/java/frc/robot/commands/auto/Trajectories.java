package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.Logger;

@SuppressWarnings("WeakerAccess")
public class Trajectories {

	public static HashMap<String, Pose2d> locations = new HashMap<String, Pose2d>();
	public static Velocity<Length> yeetSpeed = VelocityKt.getVelocity(LengthKt.getFeet(12.5)); //FIXME what is the speed for the spin?

	/**
	 * WARNING: do NOT call this more than once. it gets VERY sad
	 * TODO actually figure out why that breaks it
	 */
	private static void genLocs() {
		locations.put("habR", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684), Rotation2dKt.getDegree(180)));
		locations.put("habM", new Pose2d(LengthKt.getFeet(5.181), LengthKt.getFeet(13.379), Rotation2dKt.getDegree(180)));
		locations.put("habL", new Pose2d(LengthKt.getFeet(5.141), LengthKt.getFeet(9.508), Rotation2dKt.getDegree(180)));
		locations.put("loadingL", new Pose2d(LengthKt.getFeet(1.286), LengthKt.getFeet(25.021), Rotation2dKt.getDegree(180.0)));
		locations.put("loadingR", new Pose2d(LengthKt.getFeet(1.325), LengthKt.getFeet(2.336), Rotation2dKt.getDegree(180.0)));
		locations.put("cargoL1", new Pose2d(LengthKt.getFeet(21.565), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(0d))); // I changed this! -Matt
		// locations.put("cargoL1Rotated", new Pose2d(LengthKt.getFeet(21.565), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(-90.0)));
		locations.put("cargoL2", new Pose2d(LengthKt.getFeet(23.532), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(0d))); // I changed this! -Matt
		locations.put("cargoL3", new Pose2d(LengthKt.getFeet(25.277), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(0d))); // I changed this! -Matt
		locations.put("cargoML", new Pose2d(LengthKt.getFeet(17.101), LengthKt.getFeet(14.338), Rotation2dKt.getDegree(180)));
		locations.put("cargoMR", new Pose2d(LengthKt.getFeet(17.066), LengthKt.getFeet(12.653), Rotation2dKt.getDegree(180)));
		locations.put("cargoR1", new Pose2d(LengthKt.getFeet(21.565), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(0d))); // I changed this! -Matt
		locations.put("cargoR2", new Pose2d(LengthKt.getFeet(23.532), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(0d))); // I changed this! -Matt
		locations.put("cargoR3", new Pose2d(LengthKt.getFeet(25.277), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(0d))); // I changed this! -Matt
		locations.put("depotL", new Pose2d(LengthKt.getFeet(5.203), LengthKt.getFeet(20.517), Rotation2dKt.getDegree(0)));
		locations.put("depotR", new Pose2d(LengthKt.getFeet(5.203), LengthKt.getFeet(6.107), Rotation2dKt.getDegree(0))); //FIXME can w actually pick up cargo like this?
		locations.put("yeetL", new Pose2d(LengthKt.getFeet(13.606), LengthKt.getFeet(21.315), Rotation2dKt.getDegree(145))); //basically arb. rn
		locations.put("yeetR", new Pose2d(LengthKt.getFeet(13.606), LengthKt.getFeet(5.685), Rotation2dKt.getDegree(-145))); //basically arb. rn
		locations.put("pyeetL", new Pose2d(LengthKt.getFeet(13.606), LengthKt.getFeet(21.315), Rotation2dKt.getDegree(325))); //basically arb. rn
		locations.put("pyeetR", new Pose2d(LengthKt.getFeet(13.606), LengthKt.getFeet(5.685), Rotation2dKt.getDegree(35))); //basically arb. rn

	}

	public static HashMap<String, TimedTrajectory<Pose2dWithCurvature>> generatedHGTrajectories = new HashMap<String, TimedTrajectory<Pose2dWithCurvature>>();
	public static HashMap<String, TimedTrajectory<Pose2dWithCurvature>> generatedLGTrajectories = new HashMap<String, TimedTrajectory<Pose2dWithCurvature>>();
  public static List<String> grabs = new ArrayList<String>(Arrays.asList("habR", "habM", "habL", "loadingL", "loadingR", "depotLF", "depotLB", "depotRF", "depotRB"));
	public static List<String> puts = new ArrayList<String>(Arrays.asList("cargoL1", "cargoL2", "cargoL3", "cargoML", "cargoMR", "cargoR1", "cargoR2", "cargoR3",
			"rocketL1", "rocketL2", "rocketL3", "rocketR1", "rocketR2", "rocketR3"));

	public static final Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
	public static final Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));

	public static final Velocity<Length> kDefaultVelocity = VelocityKt.getVelocity(LengthKt.getFeet(5));
	public static final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(8));

	// public static final TimedTrajectory<Pose2dWithCurvature> forward20Feet = generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(
	//   new Pose2d(LengthKt.getFeet(0), LengthKt.getFeet(0),Rotation2dKt.getDegree(0)),
	//   new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(0),Rotation2dKt.getDegree(0)))),
	//   false);

	private static final ArrayList<Pose2d> forward20ftSrc = new ArrayList<Pose2d>(Arrays.asList(
			new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(5), Rotation2dKt.getDegree(0)),
			new Pose2d(LengthKt.getFeet(35), LengthKt.getFeet(5), Rotation2dKt.getDegree(0))));
	// new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(5), Rotation2dKt.getDegree(45)),
	// new Pose2d(LengthKt.getFeet(30), LengthKt.getFeet(5), Rotation2dKt.getDegree(-90)),
	// new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(5), Rotation2dKt.getDegree(135)),
	// new Pose2d(LengthKt.getFeet(10), LengthKt.getFeet(5), Rotation2dKt.getDegree(-90)),
	// new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(5), Rotation2dKt.getDegree(45))));
	public static TimedTrajectory<Pose2dWithCurvature> forward20Feet;

	private static List<TimingConstraint<Pose2dWithCurvature>> kLowGearConstraints = Arrays.asList(
			new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(10))),
			new DifferentialDriveDynamicsConstraint(Constants.kLowGearDifferentialDrive, 12 /* volts */),
			// This limits our velocity while within the given Rectangle2d to 2 feet per second (read: the hab)
			new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(2.0))));

	private static List<TimingConstraint<Pose2dWithCurvature>> kHighGearConstraints = Arrays.asList(
			new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(10))),
			new DifferentialDriveDynamicsConstraint(Constants.kHighGearDifferentialDrive, 12 /* volts */),
			// This limits our velocity while within the given Rectangle2d to 2 feet per second (read: the hab)
			new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(2.0))));

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

		Logger.log("Generating ALL trajectories");
		genLocs();
		double startTime = Timer.getFPGATimestamp();
		for (String key : locations.keySet()) {
			for (String eKey : locations.keySet()) {
				if (!((grabs.contains(key) && grabs.contains(eKey)) || (puts.contains(key) && puts.contains(eKey)))) {
					System.out.printf("Current start key: %s Current end key: %s\n", key, eKey);
					generatedTrajectories.put(key + " to " + eKey, //FIXME this is a terrible way to mark unique paths, but it works
							generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get(key),
									locations.get(eKey))), false));
				}
			}
		}
		// System.out.printf("Current start key: %s Current end key: %s\n", "habM", "cargoML");
		// generatedTrajectories.put("habM" + " to " + "cargoML", //FIXME this is a terrible way to mark unique paths, but it works
		// 		generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("habM"),
		// 				locations.get("cargoML"))), false));
		System.out.println("Out of first round of generation");
		double now = 0;
		if (isReal)
			now = Timer.getFPGATimestamp();
		Logger.log("Trajectories generated in " + (now - startTime) + " seconds!");
	}

	/**
	 * Generate a trajectory from a list of waypoints in high gear
	 * @param waypoints to follow
	 * @param reversed if the path is reversed
	 * @return
	 */
	public static TimedTrajectory<Pose2dWithCurvature> generateTrajectoryHighGear(List<Pose2d> waypoints, boolean reversed) {
		return generateTrajectory(waypoints, kHighGearConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocity, kDefaultAcceleration, reversed);
	}

	/**
	* Generate a trajectory from a list of waypoints in low gear
	* @param waypoints to follow
	* @param reversed if the path is reversed
	* @return
	*/
	public static TimedTrajectory<Pose2dWithCurvature> generateTrajectoryLowGear(List<Pose2d> waypoints, boolean reversed) {
		return generateTrajectory(waypoints, kLowGearConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocity, kDefaultAcceleration, reversed);
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
			List<? extends TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed) {
		return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(
				waypoints,
				constraints_,
				startVelocity,
				endVelocity,
				maxVelocity,
				maxAcceleration,
				reversed);
	}

}
