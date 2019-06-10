package frc.robot.lib.motion;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;

import frc.robot.lib.obj.RoundRotation2d;
import org.opencv.core.Point;

/**
 * Contains basic functions that are used often.
 */
public class Util {

	public static final double kEpsilon = 1e-12;

	/**
	 * Prevent this class from being instantiated.
	 */
	private Util() {}

	/**
	 * Limits the given input to the given magnitude.
	 */
	public static double limit(double v, double maxMagnitude) {
		return limit(v, -maxMagnitude, maxMagnitude);
	}

	public static double limit(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}

	public static RoundRotation2d limit(RoundRotation2d v, RoundRotation2d min, RoundRotation2d max) {
		if (v.getDegree() > max.getDegree())
			v = max;
		if (v.getDegree() < min.getDegree())
			v = min;
		return v;
	}

	public static Velocity<Length> limit(Velocity<Length> v, Velocity<Length> min, Velocity<Length> max) {
		if (v.getValue() > max.getValue())
			v = max;
		if (v.getValue() < min.getValue())
			v = min;
		return v;
	}

	public static Velocity<Length> limit(Velocity<Length> v, Velocity<Length> max) {
		return limit(v, max.times(-1), max);
	}

	public static double deadband(double v, double deadband) {
		return (Math.abs(v) < deadband) ? 0 : v;
	}

	public static double interpolate(double a, double b, double x) {
		x = limit(x, 0.0, 1.0);
		return a + (b - a) * x;
	}

	public static String joinStrings(final String delim, final List<?> strings) {
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < strings.size(); ++i) {
			sb.append(strings.get(i).toString());
			if (i < strings.size() - 1) {
				sb.append(delim);
			}
		}
		return sb.toString();
	}

	public static boolean epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean epsilonEquals(double a, double b) {
		return epsilonEquals(a, b, kEpsilon);
	}

	public static boolean epsilonEquals(int a, int b, int epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
		boolean result = true;
		for (Double value_in : list) {
			result &= epsilonEquals(value_in, value, epsilon);
		}
		return result;
	}

	public static double toFeet(double meters) {
		return meters * 3.28084;
	}

	public static double toMeters(double feet) {
		return feet * 0.3048;
	}

	public static double round(double value, int places) {
		if (places < 0)
			throw new IllegalArgumentException();

		BigDecimal bd = new BigDecimal(Double.toString(value));
		bd = bd.setScale(places, RoundingMode.HALF_UP);
		return bd.doubleValue();
	}

	public static Length limit(Length v, Length min, Length max) {
		if (v.getValue() > max.getValue())
			v = max;
		if (v.getValue() < min.getValue())
			v = min;
		return v;
	}

	public static Rotation2d limit(Rotation2d v, Rotation2d min, Rotation2d max) {
		if (v.getValue() > max.getValue())
			v = max;
		if (v.getValue() < min.getValue())
			v = min;
		return v;
	}

	public static RoundRotation2d min(RoundRotation2d bound1, RoundRotation2d bound2) {
		var min = RoundRotation2d.getDegree(Math.min(bound1.getDegree(), bound2.getDegree()));
		return min;
	}

	public static RoundRotation2d max(RoundRotation2d bound1, RoundRotation2d bound2) {
		var min = RoundRotation2d.getDegree(Math.max(bound1.getDegree(), bound2.getDegree()));
		return min;
	}

	public static Length max(Length arg1, Length arg2) {
		return LengthKt.getInch(Math.max(arg1.getInch(), arg2.getInch()));
	}

	// public static boolean isWithin(RoundRotation2d v, RoundRotation2d min, RoundRotation2d max) {
	// 	var isBelowMax = v.minus(max).getDegree() < 0;
	// 	var isAboveMin = min.minus(v).getDegree() < 0;
	// 	return isBelowMax && isAboveMin;
	// }

	public static boolean isWithin(RoundRotation2d v, RoundRotation2d bound1, RoundRotation2d bound2) {
		var max = Math.max(bound1.getDegree(), bound2.getDegree());
		var min = Math.min(bound1.getDegree(), bound2.getDegree());
		var isBelowMax = v.getDegree() - max < 0;
		var isAboveMin = min - v.getDegree() < 0;
		return isBelowMax && isAboveMin;
	}

	public static RoundRotation2d getWorstCase(RoundRotation2d worstCase, RoundRotation2d bound1, RoundRotation2d bound2) {
		// RoundRotation2d toReturn;
		if (isWithin(worstCase, bound1, bound2))
			return worstCase;
		else
			return min(bound1, bound2);
	}

	public static String toString(Pose2d pose) {
		return String.format("Pose: (%s, %s) theta: (%s)", pose.getTranslation().getX().getFeet(), pose.getTranslation().getY().getFeet(), pose.getRotation().getDegree());
	}

	public static Pose2d reflectWaypoint(Pose2d waypoint) {
		var toReturn = new Pose2d(
				// new Translation2d(LengthKt.getFeet(((13.5 - waypoint.getTranslation().getY().getFeet()) * -1) + 13.5),
				// waypoint.getTranslation().getY()),
				waypoint.getTranslation().getX(),
				LengthKt.getFeet(27).minus(waypoint.getTranslation().getY()),
				new Rotation2d(waypoint.getRotation().getRadian() * -1));

		return toReturn;

		// return new Pose2d();
	}

	public static List<Pose2d> reflectTrajectory(List<Pose2d> old_) {
		ArrayList<Pose2d> new_ = new ArrayList<>();
		for (Pose2d pose : old_) {
			Pose2d reflected = reflectWaypoint(pose);
			// new_.add(reflectWaypoint(pose));
			new_.add(reflected);
			// System.out.println(toString(reflected));
		}
		// return new_;
		return new_.subList(0, new_.size());
	}

	public static double interpolate(Point a, Point b, double x) {
		double slope = (b.y - a.y) / (b.x - a.x);
		double intercept = a.y - (slope * a.x);
		return slope * x + intercept;
	}

}
