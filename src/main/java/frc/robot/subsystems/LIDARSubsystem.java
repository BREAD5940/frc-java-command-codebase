package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.command.Subsystem;

public class LIDARSubsystem extends Subsystem {
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new LIDARCommand());
  }
  
  private static final int CALIBRATION_OFFSET = -6;
  
  private Counter counter;
  private int printedWarningCount = 0;
  
  /**
  * Create an object for a LIDAR-Lite attached to some digital input on the roboRIO
  * @param source The DigitalInput or DigitalSource where the LIDAR-Lite is attached (ex: new DigitalInput(9))
  */
  public void initLIDAR(DigitalSource source) {
    counter = new Counter(source);
    counter.setMaxPeriod(1.0);
    // Configure for measuring rising to falling pulses
    counter.setSemiPeriodMode(true);
    counter.reset();
  }
  
  /**
  *  Used to get distance (of what?) in cm
  * @return Distance in inches
  */
  public double getDistanceCm() {
    // If we haven't seen the first rising to falling pulse, then we have no measurement.
    // This happens when there is no LIDAR-Lite plugged in, btw.
    if (counter.get() < 1) {
      if (++printedWarningCount < 5) {
        System.out.println("LidarLitePWM: waiting for distance measurement");
      }
      return 0;
    }
    /* getPeriod returns time in seconds. The hardware resolution is microseconds.
    * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
    */
    return (counter.getPeriod() * 1000000.0 / 10.0) + CALIBRATION_OFFSET;
  }
  public long getDistanceCmRounded() { return Math.round(getDistanceCm()); }
  /**
  * Used to get distance (of what?) in Inches
  * @return Distance in inches
  */
  public double getDistanceIn() { return getDistanceCm() * 0.3937007874; }
  public long getDistanceInRounded() { return Math.round(getDistanceIn()); }
}
