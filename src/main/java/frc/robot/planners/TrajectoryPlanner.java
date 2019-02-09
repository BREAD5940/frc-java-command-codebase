package frc.robot.planners;

import frc.robot.lib.Path;

public class TrajectoryPlanner {

	protected static double maxV;
	protected static double epsiWeight;
	protected static double maxVOffWeight;
	protected static double deltaWeight;
	protected static double accelWeight;
	protected static double deltaAccelWeight;
	protected static double deltaDeltaWeight;
	protected Path path;

	public TrajectoryPlanner(Path path) {
		this.path = path;
		/*
		  this should do the following:
		  1. take an input of (1) maximum/minimum angles, (2) maximum/minimum heights, (3) how stuff should be weighted, (4) the desired Path
		  2. Somehow convert it into a kinematic model
		a. This is gonna be a ton of math because its up-down-angle-angle as opposed to back-forwards-left-right
		 */

	}

}
