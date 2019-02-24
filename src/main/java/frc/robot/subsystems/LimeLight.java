package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotConfig;

/**
 * tv  Whether the limelight has any valid targets (0 or 1)
 * tx  Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
 * ty  Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
 * ta  Target Area (0% of image to 100% of image)
 * ts  Skew or rotation (-90 degrees to 0 degrees)
 * tl  The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
 * TODO Move this to Pantry Vision
 * 
 * @author Matthew Morley
 */
public class LimeLight {

	private static LimeLight instance;
	private static Object mutex = new Object();

	public static LimeLight getInstance() {
		LimeLight result = instance;
		if (result == null) {
			synchronized (mutex) {
				result = instance;
				if (result == null)
					instance = result = new LimeLight();
			}
		}
		return result;
	}

	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	double[] data, angles;
	double cameraHeight = RobotConfig.limeLight.camera_height;
	double cameraAngle = RobotConfig.limeLight.camera_angle;
	double x_resolution = 320;
	double y_resolution = 240;
	double x_fov = 54;
	double y_fov = 41;
	double x_focal_length = x_resolution / (2 * Math.tan(x_fov / 2));
	double y_focal_length = y_resolution / (2 * Math.tan(y_fov / 2));
	double average_focal_length = (x_focal_length + y_focal_length) / 2;

	double distance, relativeAngle;

	public double[] getData() {
		/** Whether the limelight has any valid targets (0 or 1) */
		NetworkTableEntry tv = table.getEntry("tv");
		/** Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
		NetworkTableEntry tx = table.getEntry("tx");
		/** Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
		NetworkTableEntry ty = table.getEntry("ty");
		/** Target Area (0% of image to 100% of image) */
		NetworkTableEntry ta = table.getEntry("ta");
		/** Skew or rotation (-90 degrees to 0 degrees) */
		NetworkTableEntry ts = table.getEntry("ts");
		/** The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency. */
		NetworkTableEntry tl = table.getEntry("tl");

		double[] data = new double[6];

		data[0] = tv.getDouble(0);
		data[1] = tx.getDouble(0);
		data[2] = ty.getDouble(0);
		data[3] = ta.getDouble(0);
		data[4] = ts.getDouble(0);
		data[5] = tl.getDouble(0);
		return data;
	}

	/** Turn on the Limelight LED */
	public void turnOnLED() {
		table.getEntry("ledMode").setNumber(0);
	}

	/** Turn off the Limelight LED */
	public void turnOffLED() {
		table.getEntry("ledMode").setNumber(1);
	}

	/**
	 * Get the area of the tracked target as a percentage from 0% to 100%
	 * @return area as percentage of total area 
	 */
	public double getTargetArea() {
		return (table.getEntry("ta")).getDouble(0);
	}

	/**
	 * Get the dx from crosshair to tracked target
	 * @return skew from -90 to 0 degrees
	 */
	public double getTargetSkew() {
		return (table.getEntry("ts")).getDouble(0);
	}

	/**
	 * Get the latency from camera data to NT entry
	 * @return pipeline latency contribution
	 */
	public double getPipelineLatency() {
		return (table.getEntry("tl")).getDouble(0);
	}

	/**
	 * Get the dx from crosshair to tracked target
	 * @return dx
	 */
	public double getDx() {
		return (table.getEntry("tx")).getDouble(0);
	}

	/**
	 * Get the dy from crosshair to tracked target
	 */
	public double getDy() {
		return (table.getEntry("ty")).getDouble(0);
	}

	/**
	 * Return the number of targets currently being tracked
	 * @return currently tracked targets
	 */
	public double getTrackedTargets() {
		return (table.getEntry("tv")).getDouble(0);
	}

	/**
	 * Get the current delta x (left/right) angle from crosshair to vision target
	 * @return delta x in degrees to target
	 */
	public double getDxAngle() {
		return Math.toDegrees(
				Math.atan(
						getDx() / average_focal_length));
	}

	/**
	 * Get the current elevation (delta y) angle from crosshair to vision target
	 * @return degrees of elevation from crosshair to target 
	 */
	public double getDyAngle() {
		return Math.toDegrees(
				Math.atan(
						getDy() / average_focal_length));
	}

	/**
	 * Get the distance (in the same units as elevation) from a tracked vision target
	 * @param targetElevation
	 * @return distance to target
	 */
	public double getDistanceToFixedPoint(double targetElevation) {
		return (targetElevation - cameraHeight) / Math.tan(cameraAngle + (table.getEntry("ty").getDouble(0)));
	}

}
