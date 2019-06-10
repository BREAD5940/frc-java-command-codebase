package frc.robot.subsystems;

import frc.robot.lib.obj.RoundRotation2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.Time;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotConfig;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.VisionTarget;
import frc.robot.lib.obj.factories.VisionTargetFactory;

import static java.lang.Math.PI;

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
public class LimeLight extends Subsystem {

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

	NetworkTable table;
	double[] data, angles;
	double cameraHeight = RobotConfig.limeLight.camera_height;
	double cameraAngle = RobotConfig.limeLight.camera_angle;
	private static final double x_resolution_low = 320;
	private static final double y_resolution_low = 240;
	private static final double x_resolution_high = 960;
	private static final double y_resolution_high = 720;
	private static final Rotation2d x_fov = Rotation2dKt.getDegree(59.6);
	private static final Rotation2d y_fov = Rotation2dKt.getDegree(45.7);
	private static final double x_focal_length_low = x_resolution_low / (2 * Math.tan(x_fov.getRadian() / 2));
	private static final double y_focal_length_low = y_resolution_low / (2 * Math.tan(y_fov.getRadian() / 2));
	private static final double x_focal_length_high = x_resolution_low / (2 * Math.tan(x_fov.getRadian() / 2));
	private static final double y_focal_length_high = y_resolution_low / (2 * Math.tan(y_fov.getRadian() / 2));
	boolean isHighRes = false;
	public PipelinePreset mCurrentPipeline;
	private static final PipelinePreset kDefaultPreset = PipelinePreset.k2dVision;

	private static final VisionTarget kRocketCargoSingleTarget = VisionTargetFactory.getRocketCargoSingleTarget();
	private static final VisionTarget kHatchSingleTarget = VisionTargetFactory.getHatchSingleTarget();
	private static final VisionTarget kRocketCargoDualTarget = VisionTargetFactory.getRocketCargoDualTarget();
	private static final VisionTarget kHatchDualTarget = VisionTargetFactory.getHatchDualTarget();
	private static final int kDefaultPipeline = 1;
	private NetworkTable smartDashboard;

	private LimeLight() {
		this.table = NetworkTableInstance.getDefault().getTable("limelight");
		this.setPipeline(kDefaultPreset);
		smartDashboard = NetworkTableInstance.getDefault().getTable("SmartDashboard");
		smartDashboard.getEntry("Desired Vision Pipeline").setNumber(kDefaultPipeline);
		smartDashboard.addEntryListener("Desired Vision Pipeline",
				(smartDashboard, key, entry, value, flags) -> {
					setPipeline((int) value.getDouble());
					System.out.println("Value changed! it's now " + (int) value.getDouble());
				},
				EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		table.getEntry("stream").setNumber(2);

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

	public double getTargetXPixels() {
		return table.getEntry("thor").getDouble(0);
	}

	/** Turn off the Limelight LED */
	public void turnOffLED() {
		table.getEntry("ledMode").setNumber(1);
	}

	/**
	 * Get the Pose2d of a vision target with an offset applied. This way someone else can account for offset from limelight to bumpers
	 * @param kOffset how far away the front of the bumpers is from the camera (in inches)
	 * @param distanceToShiftBy how far to move everything up/right so it shows up on falcon dashboard
	 */
	public Pose2d getPose(double distanceToShiftBy) {
		double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});

		// final double kOffset = 100;

		// LinearDigitalFilter

		// final double kLimelightForeOffset = 25; //inches from limelight to hatch pannel
		// forward/backward motion, left/right motion
		Translation2d mTranToGoal = new Translation2d(LengthKt.getInch((camtran[2]) + distanceToShiftBy), LengthKt.getInch((camtran[0] * -1) + distanceToShiftBy));
		Rotation2d mRotToGoal = Rotation2dKt.getDegree(camtran[4] * 1);
		Pose2d mPoseToGoal = new Pose2d(mTranToGoal, mRotToGoal);

		System.out.println(Util.toString(mPoseToGoal));

		return mPoseToGoal;
	}

	public enum PipelinePreset {
		kDefault(2), k2dVision(1), k3dVision(0);

		private int id;

		private static PipelinePreset[] values = null;

		public static PipelinePreset fromID(int i) {
			if (PipelinePreset.values() == null) {
				PipelinePreset.values = PipelinePreset.values();
			}
			return PipelinePreset.values[i];
		}

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
		if (req_.name().contains("3d")) {
			this.isHighRes = true;
		} else {
			this.isHighRes = false;
		}
	}

	public void setPipeline(int req_) {
		if (req_ > PipelinePreset.values().length - 1)
			return;
		PipelinePreset _req_ = PipelinePreset.values()[req_];
		this.mCurrentPipeline = _req_;
		setPipeline(_req_);
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
	 * @return pipeline latency contribution in seconds
	 */
	public Time getPipelineLatency() {
		return TimeUnitsKt.getMillisecond((table.getEntry("tl").getDouble(0) / 1000) + 11);
	}

	/**
	 * Get the dx from crosshair to tracked target
	 * @return dx
	 */
	public Rotation2d getDx() {
		return kRotation2d.createNew((table.getEntry("tx")).getDouble(0));
	}

	private static final Rotation2d kRotation2d = Rotation2dKt.getDegree(0);

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

	/**
	 * Estimate the distance to a given target using a measurement
	 * @param kTarget the target we are tracking
	 * @param mMeasured the target we measured
	 */
	public Length getDistance(VisionTarget kTarget, MeasuredVisionTarget mMeasured) {
		Length width = kTarget.getWidth();
		double focalLen = (isHighRes) ? x_focal_length_high : x_focal_length_low;
		Length distance = width.times(focalLen).div(mMeasured.getX());
		return distance;
	}

	/**
	 * Get the distance to the target using X axis size estimation
	 * @return the distance to the target, maybe
	 */
	public Length getDistanceToTarget() {
		double focalLen = (isHighRes) ? x_focal_length_high : x_focal_length_low;
		Length width = LengthKt.getInch(14.6);
		double targetSizePx = getTargetXPixels();
		Length distance = width.times(focalLen).div(targetSizePx);
		return distance;
	}

	public class MeasuredVisionTarget {
		private double x_, y_, width_, height_, area_;

		MeasuredVisionTarget(double x, double y, double width, double height, double area) {
			x_ = x;
			y_ = y;
			width_ = width;
			height_ = height;
			area_ = area;
		}

		public double getX() {
			return x_;
		}

		public double getY() {
			return y_;
		}

		public double getWidth() {
			return width_;
		}

		public double getHeight() {
			return height_;
		}

		public double getArea() {
			return area_;
		}
	}

	public enum LEDMode {
		kON, kOFF;
	}

	public static class SetLEDs extends InstantCommand {
		private LEDMode mode;

		public SetLEDs(LEDMode mode) {
			this.mode = mode;
			setRunWhenDisabled(true);
		}

		@Override
		protected void initialize() {
			if (mode == LEDMode.kON)
				LimeLight.getInstance().turnOnLED();
			if (mode == LEDMode.kOFF)
				LimeLight.getInstance().turnOffLED();
		}

	}

	public static class setPipeline extends InstantCommand {
		private PipelinePreset mode;

		public setPipeline(PipelinePreset mode) {
			this.mode = mode;
			setRunWhenDisabled(true);
		}

		@Override
		protected void initialize() {
			LimeLight.getInstance().setPipeline(mode);
		}

	}

	@Override
	protected void initDefaultCommand() {

	}

	public Length estimateDistanceFromAngle() {
		final Rotation2d cameraAngle = Rotation2dKt.getDegree(-29);
		final Length cameraHeight = LengthKt.getInch(30); // TODO check me
		final Length targetHeight = LengthKt.getInch(29); // ????? for hatches only

		var horizonalDelta = targetHeight.minus(cameraHeight);
		return horizonalDelta.div(
				Math.tan(
						cameraAngle.getRadian() + getDy().getRadian() * (PI / 180d)
				)
		);

//
//		Rotation2d targetAngle = getDy().plus(cameraAngle);
//
//		var distance = (visionTargetHeight.minus(cameraHeight)).div(Math.tan(targetAngle.getRadian()));
//
//		System.out.println("estimated distance: " + distance.getInch());
//
//		return distance;

	}

	public boolean hasTarget() {
		return getTrackedTargets() > 0.5;
	}

	@Override
	public void periodic() {
		// var target = VisionTargetFactory.getHatchDualTarget();
		// var measured = new MeasuredVisionTarget
		// var distance = getDistance(target, mMeasured)
	}

}
