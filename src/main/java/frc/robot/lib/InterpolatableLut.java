package frc.robot.lib;

import java.util.TreeMap;

public class InterpolatableLut {

	// ArrayList<InterpolatableLutEntry> entries;
	TreeMap<Double, InterpolatableLutEntry> map;

	public InterpolatableLut(TreeMap<Double, InterpolatableLutEntry> map) {
		this.map = map;
	}

	public Double interpolate(Double key) {

		var topBound = map.ceilingEntry(key);
		var bottomBound = map.floorEntry(key);

		if (topBound == null)
			return bottomBound.getValue().getValue();
		else if (bottomBound == null)
			return topBound.getValue().getValue();
		else
			return bottomBound.getValue().interpolate(topBound.getValue().getValue(), (key - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey()));

	}

}
