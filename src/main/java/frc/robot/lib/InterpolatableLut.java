package frc.robot.lib;

import java.util.TreeMap;

import frc.robot.lib.motion.Util;

public class InterpolatableLut {

	// ArrayList<InterpolatableLutEntry> entries;
	TreeMap<Double, InterpolatableLutEntry> map;

	public InterpolatableLut(TreeMap<Double, InterpolatableLutEntry> map) {
		this.map = map;
	}

	public Double interpolate(Double key) {
		// System.out.println("---------------------------------------------");
		// System.out.println("key input is " + key);

		var topBound = map.ceilingEntry(key);
		var bottomBound = map.floorEntry(key);

		// System.out.println("TOP BOUND key: " + topBound.getKey());

		// System.out.println("BOTTOM BOUND key: " + bottomBound.getKey());

		// System.out.println("TOP BOUND value: " + topBound.getValue().getValue());

		// System.out.println("BOTTOM BOUND value: " + bottomBound.getValue().getValue());

		if (topBound == null || Double.isNaN(topBound.getValue().getValue())
				|| Util.epsilonEquals(topBound.getKey(), key)) {
			// System.out.println("top bound is nan or null, returning " + bottomBound.getValue().getValue());
			return bottomBound.getValue().getValue();
		} else if (bottomBound == null || Double.isNaN(bottomBound.getValue().getValue())
				|| Util.epsilonEquals(topBound.getKey(), key)) {
			// System.out.println("bottom bound is nan or null, returning " + topBound.getValue().getValue());
			return topBound.getValue().getValue();
		} else {
			var ratio = (key - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey());
			// System.out.println("the ratio between the two points is " + ratio);
			// System.out.println("returning " + bottomBound.getValue().interpolate(topBound.getValue().getValue(), ratio));
			return bottomBound.getValue().interpolate(topBound.getValue().getValue(), ratio);
		}

	}

}
