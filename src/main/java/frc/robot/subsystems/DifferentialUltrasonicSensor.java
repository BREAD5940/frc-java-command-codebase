package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.RobotConfig.ultrasonicSensors;;



public class DifferentialUltrasonicSensor {

  public static DifferentialUltrasonicSensor instance_ = new DifferentialUltrasonicSensor();
  
  public synchronized static DifferentialUltrasonicSensor getInstance() {
    return instance_;
  }

  Ultrasonic sensor1, sensor2;

  /**
   * Set the rangemode of the sensors
   */
  public enum RangeMode {
    INCHES, MM, METER, FEET
  }
  RangeMode mMode = RangeMode.INCHES;

  /**
   * Instantiate two ultrasonic sensors with pre-defined ports
   * and automatic mode enabled. The units defaults to inches.
   */
  public DifferentialUltrasonicSensor() {
    sensor1 = new Ultrasonic( ultrasonicSensors.sensor1.echoPort , 
                            ultrasonicSensors.sensor1.triggerPort ); 
    sensor2 = new Ultrasonic( ultrasonicSensors.sensor2.echoPort , 
                            ultrasonicSensors.sensor2.triggerPort ); 
    sensor1.setAutomaticMode(true);
    sensor2.setAutomaticMode(true);

  }

  public void setMode(RangeMode mode) {
    mMode = mode;
  }

  public double getRangeSensor1() { return getDistance(sensor1, mMode); }

  public double getRangeSensor2() { return getDistance(sensor2, mMode); }

  public double[] getRangesAsDouble() { return new double[]{getDistance(sensor1, mMode), getDistance(sensor2, mMode)}; }

  public double getRangeSensor1(RangeMode mode) { return getDistance(sensor1, mode); }

  public double getRangeSensor2(RangeMode mode) { return getDistance(sensor2, mode); }

  public double[] getRangeAsDouble(RangeMode mode) { return new double[]{getDistance(sensor1, mode), getDistance(sensor2, mode)}; }

    /** To get the angle offset from the wall, we use some basic trig. The distance
    * between the sensors is a known constant, and so we construct a right angle triangle
    * with one base as the the distance and second leg as the delta between the two ranges.
    * The pose theta of the robot is hense the same as the angle between the robot width
    * and the hypotinuse, and trig gives us that tan(theta) = opposite / adjacent, or 
    * delta_distance / sensor_width, or theta is tan^{-1} delta_d / sensor_width
    * 
    * @return theta, where positive is more clockwise
    */
  public double getAngleOffset() {
    double delta = getDistance(sensor2, RangeMode.INCHES) - getDistance(sensor1, RangeMode.INCHES);
    double theta = Math.toDegrees( Math.atan( delta / ultrasonicSensors.sensorCenterDistance ));
    return theta;
  }

  private double getDistance(Ultrasonic sensor, RangeMode mMode) {

    double distance = 0;

    if (mMode == RangeMode.INCHES) {
      distance = sensor.getRangeInches();
    }
    else if (mMode == RangeMode.FEET) {
      distance = sensor.getRangeInches() / 12;
    }
    else if (mMode == RangeMode.MM) {
      distance = sensor.getRangeMM();
    }
    else if (mMode == RangeMode.INCHES) {
      distance = sensor.getRangeMM() / 1000;
    }
    else {
      throw new IllegalArgumentException("RangeMode is not a valid option");
    }

    return distance;
  }

}