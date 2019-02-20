package frc.robot.subsystems;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotConfig;
import frc.robot.lib.obj.VisionTarget;
import frc.robot.lib.obj.factories.VisionTargetFactory;

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
	NetworkTable table;
	double[] data, angles;
	double cameraHeight = RobotConfig.limeLight.camera_height;
	double cameraAngle = RobotConfig.limeLight.camera_angle;
	private static final double x_resolution_low = 320;
	private static final double y_resolution_low = 240;
	private static final double x_resolution_high = 960;
	private static final double y_resolution_high = 720;
	private static final double x_fov = 59.6;
	private static final double y_fov = 45.7;
	private static final double x_focal_length_low = x_resolution_low / (2 * Math.tan(x_fov / 2));
	private static final double y_focal_length_low = y_resolution_low / (2 * Math.tan(y_fov / 2));
	private static final double x_focal_length_high = x_resolution_low / (2 * Math.tan(x_fov / 2));
	private static final double y_focal_length_high = y_resolution_low / (2 * Math.tan(y_fov / 2));
	boolean isHighRes = false;
	public PipelinePreset mCurrentPipeline;
	private static final PipelinePreset kDefaultPreset = PipelinePreset.kCargoClose;

	private static final VisionTarget kRocketCargoTarget = VisionTargetFactory.getRocketCargoTarget();
	private static final VisionTarget kHatchTarget = VisionTargetFactory.getHatchTarget();

	public LimeLight() {
		this.table = NetworkTableInstance.getDefault().getTable("limelight");
		this.setPipeline(kDefaultPreset);
	}

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

	public enum PipelinePreset {
		kDefault(0), kCargoFar(1), kCargoClose(2), kCargoFarHighRes(3), kCargoCloseHighRes(4);

		private int id;

		private PipelinePreset(int id) {
			this.id = id;
		}

		public int getId() {
			return id;
		}
	}

	/**
	 * Set the pipeline of the limelight
	 */
	public void setPipeline(PipelinePreset req_) {
		table.getEntry("pipeline").setNumber(req_.getId());
		this.mCurrentPipeline = req_;
		if (req_.name().contains("HighRes")) {
			this.isHighRes = true;
		} else {
			this.isHighRes = false;
		}
	}

	public int getPipeline() {
		return table.getEntry("pipeline").getNumber(0).intValue();
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
	public Rotation2d getDx() {
		return Rotation2dKt.getDegree((table.getEntry("tx")).getDouble(0));
	}

	/**
	 * Get the dy from crosshair to tracked target
	 */
	public Rotation2d getDy() {
		return Rotation2dKt.getDegree((table.getEntry("ty")).getDouble(0));
	}

	/**
	 * Return the number of targets currently being tracked
	 * @return currently tracked targets
	 */
	public double getTrackedTargets() {
		return (table.getEntry("tv")).getDouble(0);
	}

	public Length getDistance(VisionTarget target, Rotation2d yawAngle) {
		return null;
	}
	

}
