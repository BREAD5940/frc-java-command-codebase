package frc.robot.lib;

/**
 * Pretty terrible PID controller, tbh. Runs at 50 hz and currently has kp and ki
 * support. TODO is arbitrary feedforward for things like wrists and derivative
 * gains. Used in nested velocity/position PID on the drivetrian.
 * 
 * @author Matthew Morley
 */
public class TerriblePID {

  public enum FeedForwardMode {
    LINEAR, SINE, COSINE;
  }
  public enum FeedForwardBehavior {
    NORMAL, ALWAYSPOSITIVE, INVERTED
  }

  double kp, ki, kd, kf, pOutput, iOutput, fOutput, iAccum, dOutput, integralZone, 
    maxIntegralAccum, minOutput, maxOutput, setpoint, input, error, output;
  double dt = 1 / 50; // Set delta time to 50 hz, this is likely good enough
  FeedForwardMode feedforwardmode;
  FeedForwardBehavior feedforwardbehavior;
  /** The number of input units per quarter sine/cosine wave */
  double unitsPerQuarterWave;

  /**
   * Create a basic PI controller, sans the derivative term. When |error| < integralZone, 
   * the Integral term will be active. If this is no longer true, the interal accum will
   * be flushed and the controller will effectively be a P controller. Slightly less
   * shitty version of drivetrain.shitty_p_loop :P
   * @param kp gain
   * @param ki gain
   * @param integralZone about which integral will be active
   * @param maxIntegralAccum same as minimum accum, i term will top/bottom out here
   */
  public TerriblePID(double kp, double ki, double minOutput, double maxOutput, 
    double integralZone, double maxIntegralAccum) {
      this.kp = kp;
      this.ki = ki;
      this.minOutput = minOutput;
      this.maxOutput = maxOutput;
      this.integralZone = integralZone;
      this.maxIntegralAccum = maxIntegralAccum;
  }

  /**
   * Create a basic PI controller, sans the derivative term. When |error| < integralZone, 
   * the Integral term will be active. If this is no longer true, the interal accum will
   * be flushed and the controller will effectively be a P controller. Slightly less
   * shitty version of drivetrain.shitty_p_loop :P 
   * @param kp gain
   * @param ki gain
   * @param integralZone about which integral will be active
   */
  public TerriblePID(double kp, double minOutput, double maxOutput) {
      this.kp = kp;
      this.ki = 0;
      this.minOutput = minOutput;
      this.maxOutput = maxOutput;
      this.integralZone = 0;
      this.maxIntegralAccum = 0;
  }

  /**
   * Create a basic PI controller, sans the derivative term. When |error| < integralZone, 
   * the Integral term will be active. If this is no longer true, the interal accum will
   * be flushed and the controller will effectively be a P controller. Slightly less
   * shitty version of drivetrain.shitty_p_loop :P 
   * This constructor also has support for different "shapes" of feedForward term. Depending on the 
   * @param kp gain
   * @param ki gain
   * @param integralZone about which integral will be active
   * @param maxIntegralAccum same as minimum accum, i term will top/bottom out here
   */
  public TerriblePID(
    double kp,
    double ki,
    double minOutput,
    double maxOutput,
    double integralZone,
    double maxIntegralAccum,
    double kf,
    FeedForwardMode feedforwardmode,
    FeedForwardBehavior feedforwardbehavior,
    double unitsPerQuarterWave
  ) {
      this.kp = kp;
      this.ki = ki;
      this.minOutput = minOutput;
      this.maxOutput = maxOutput;
      this.integralZone = integralZone;
      this.maxIntegralAccum = maxIntegralAccum;
      this.feedforwardmode = feedforwardmode;
      this.feedforwardbehavior = feedforwardbehavior;
      this.kf = kf;
      this.unitsPerQuarterWave = unitsPerQuarterWave;
  }
  /**
   * Set the setpoint for this instance of the PID loop. Should be preserved.
   * @param setpoint
   */
  public void setSetpoint(double demand) {
    setpoint = demand;
  }

  /** Set the kp gain of the controller */
  public void setKpGain(double kpGain) {
    kp = kpGain;
  }

  /**
   * Set the maximum output of the controller
   * @param maxOutput
   */
  public void setMaxOutput(double max) {
    maxOutput = max;
  }

  /**
   * Returns the current setpoint of the PID loop
   * @return setpoint
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Get the last calculated error of the PID loop
   * @return error
   */
  public double getError() {
    return error;
  }

  /**
   * Get the last calculated output of the PID loop
   * @return output
   */
  public double getOutput() {
    return output;
  }

  /**
   * Clamps the integral accumulator to the max output/min accum as set up on class construction
   * @param iAccum
   * @return iAccum but clamped
   */
  private double clampIntegral(double i) {
    if ( i > maxIntegralAccum ) {
      i = maxIntegralAccum;
    }
    else if ( i < (-1 * maxIntegralAccum)) {
      i = -1 * maxIntegralAccum;
    }
    return i;
  }

  /**
   * Clamps the output to the max output/min output set up on class construction
   * @param output
   * @return output but clamped
   */
  private double clampOutput(double output) {
    if ( output > maxOutput ) {
      return maxOutput;
    }
    else if ( output < minOutput) {
      return minOutput;
    }
    else {
      return output;
    }
  }

  /**
   * Calculates the feed forward term based on set settings
   */
  private double calculateFeedForward(double input) {
    switch (feedforwardmode) {
      case SINE:
        fOutput = Math.sin(Math.toRadians( input * unitsPerQuarterWave ));
      case COSINE:
        fOutput = Math.cos(Math.toRadians( input * unitsPerQuarterWave ));
      default:
        fOutput = input * kf;
    }
    switch (feedforwardbehavior) {
      case ALWAYSPOSITIVE:
        fOutput = Math.abs(fOutput);
      case INVERTED:
        fOutput = fOutput * -1;
      case NORMAL:
        break;
    }
    return fOutput;
  }

  /**
   * Updates the PIF(sans D) loop taking only the measured param. It will
   * calculated the p term, incrament the i accum, and clamp it, and return
   * the clamped output
   * @param measured
   * @return output
   */
  public double update(double measured) {
    error = setpoint - measured;

    /**
     * P output is just the error times porportional gain
     */
    pOutput = kp * error;

    /**
     * The iAccum should start at 0, but is incramented by error 
     * times dt. This is then clamped to the minimum/maximum of 
     * the i term. This only happens if the integral gain is set.
     */
    if(ki != 0) {
      iAccum += error * ki * dt; // incrament the I term by error times integral gain times delta time (numerical integration yeet)
      iAccum = clampIntegral(iAccum + iOutput); // clamp the term to the min/max   
    }

    if(kf != 0) {
      fOutput = calculateFeedForward(measured);
    }

    /**
     * This will make sure the output of the loop does not exceed the specified min/max.
     */
    output = clampOutput(pOutput + iAccum  + fOutput); 

    return output;
  }
}
