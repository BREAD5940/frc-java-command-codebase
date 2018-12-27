package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;


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
  double[] data;

  public double[] getData() {
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");
    NetworkTableEntry tl = table.getEntry("tl");
    data[0] = tv.getDouble(0);
    data[1] = tx.getDouble(0);
    data[2] = ty.getDouble(0);
    data[3] = ta.getDouble(0);
    data[4] = ts.getDouble(0);
    data[5] = tl.getDouble(0);
    return data;
  }
}