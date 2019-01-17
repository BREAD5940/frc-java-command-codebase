package frc.robot.lib.motion.purepursuit;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class Path {
	public List<WayPoint2> waypoints;
	public double spacing;
	public double maxVelocity;
	public double maxAcceleration;
	public double maxAngleVel;
	public double lookAheadDistance;

	//
	public int[] injectionSteps;
	double numFinalPoints = 0;// Don't touch this, it's in 1712's code algorithm

	@Deprecated
	public Path(List<WayPoint2> waypoints, double spacing) {
		this.waypoints = waypoints;
		this.spacing = spacing;
	}

	public Path(double spacing, double maxVelocity, double maxAcceleration, double maxAngleVel,
			List<WayPoint2> waypoints) {
		this.waypoints = waypoints;
		this.spacing = spacing;
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.maxAngleVel = maxAngleVel;
	}

	public Path(Path path) {
		this.spacing = path.spacing;
		this.maxVelocity = path.maxVelocity;
		this.maxAcceleration = path.maxAcceleration;
		this.maxAngleVel = path.maxAngleVel;
		this.waypoints = new ArrayList<WayPoint2>();
		waypoints.forEach((WayPoint2 waypoint) -> {
			this.waypoints.add(waypoint.copy());
		});
	}

	public Path copy() {
		Path newPath = new Path(spacing, maxVelocity, maxAcceleration, maxAngleVel, new ArrayList<WayPoint2>());

		waypoints.forEach((WayPoint2 waypoint) -> {
			newPath.waypoints.add(waypoint.copy());
		});
		return newPath;
	}

	public Path readFromFile(String filePath) {
		File traj = new File("/home/lvuser/deploy/paths/test.pf1.csv");
		Trajectory pathfinderTraj = Pathfinder.readFromCSV(traj);
		
		return readFromPathfinder(pathfinderTraj);
	}

	public Path readFromPathfinder(Trajectory mSource) {
		// return a new Path instance which contains all the settings that this was 
		// instantiated with, but with x,y taken from pathfinder
		ArrayList<WayPoint2> newPath = new ArrayList<WayPoint2>();

		// TODO check if these are constants that can be called from the constructor instead?
		double maxSpacing, maxSpeed, maxAccel;

		for(int i=0; i<mSource.length(); i++) {
			// Each waypoint must hold a Point x,y, and a velocity.
			// Curvature and distance is ommited from this.
			newPath.add(i, new WayPoint2(mSource.get(i).x, mSource.get(i).y, mSource.get(i).velocity));

			// TODO check for max values
			// if ((i > 0) && (i <= mSource.length()) && (mSource.get(i).distance > maxDelta) ){

			// }
		}

		return null;

	}




	@Override
	public String toString() {
		String str = "";
		for (int i = 0; i < waypoints.size(); i++) {
			str += waypoints.get(i).toString() + "\n";
		}
		return str;
	}
}
// Trajectory.Config config = new
// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
// Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
// Waypoint[] points = new Waypoint[] {
// new Waypoint(-4, -1, Pathfinder.d2r(-45)),
// new Waypoint(-2, -2, 0),
// new Waypoint(0, 0, 0)
// };