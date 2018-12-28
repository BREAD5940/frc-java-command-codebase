package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

import frc.robot.RobotConfig;


/**
 * tv	Whether the limelight has any valid targets (0 or 1)
 * tx	Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
 * ty	Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
 * ta	Target Area (0% of image to 100% of image)
 * ts	Skew or rotation (-90 degrees to 0 degrees)
 * tl	The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
 */
public class LimeLight {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  double[] data, angles;
  double cameraHeight = RobotConfig.limeLight.camera_height;
  double cameraAngle = RobotConfig.limeLight.camera_angle;
  double x_resolution = 320;
  double y_resolution = 240;
  double x_fov = 54;
  double y_fov = 41;

  double distance, relativeAngle;

  public double[] getData() {
    NetworkTableEntry tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
    NetworkTableEntry tx = table.getEntry("tx"); // tx	Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    NetworkTableEntry ty = table.getEntry("ty"); // ty	Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    NetworkTableEntry ta = table.getEntry("ta"); // ta	Target Area (0% of image to 100% of image)
    NetworkTableEntry ts = table.getEntry("ts"); // ts	Skew or rotation (-90 degrees to 0 degrees)
    NetworkTableEntry tl = table.getEntry("tl"); // tl	The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
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
   * Get the latency from photon -> NT entry
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
    return Math.atan(
      getDx() / (x_resolution / 2) / (Math.tan(x_fov / 2))
    );
  }

  /**
   * Get the current elevation (delta y) angle from crosshair to vision target
   * @return degrees of elevation from crosshair to target 
   */
  public double getDyAngle() {
    return Math.atan(
      getDy() / (y_resolution / 2) / (Math.tan(y_fov / 2))
    );
  }

  /**
   * Get the distance (in the same units as elevation) from a tracked vision target
   * @param targetElevation
   * @return distance to target
   */
  public double getDistanceToFixedPoint(double targetElevation){
    return (targetElevation - cameraHeight) / Math.tan(cameraAngle + (table.getEntry("ty").getDouble(0)));
  }

}