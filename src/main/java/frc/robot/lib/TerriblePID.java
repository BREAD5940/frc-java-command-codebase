package frc.robot.lib;

/**
 * Pretty terrible PID controller, tbh. Runs at 50 hz and currently has kp and ki
 * support. TODO is arbitrary feedforward for things like wrists and derivative
 * gains. Used in nested velocity/position PID on the drivetrian.
 * 
 * @author Matthew Morley
 */
public class TerriblePID {

  double kp, ki, kd, pOutput, iOutput, iAccum, dOutput, integralZone, 
    maxIntegralAccum, minOutput, maxOutput, setpoint, input, error, output;
  double dt = 1 / 50; // Set delta time to 50 hz, this is likely good enough

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
   * @param maxIntegralAccum same as minimum accum, i term will top/bottom out here
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
   * Set the setpoint for this instance of the PID loop. Should be preserved.
   * @param setpoint
   */
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  /** Set the kp gain of the controller */
  public void setKpGain(double kp) {
    this.kp = kp;
  }

  /**
   * Set the maximum output of the controller
   * @param maxOutput
   */
  public void setMaxOutput(double max) {
    this.maxOutput = max;
  }

  /**
   * Returns the current setpoint of the PID loop
   * @return setpoint
   */
  public double getSetpoint() {
    return this.setpoint;
  }

  /**
   * Get the last calculated error of the PID loop
   * @return error
   */
  public double getError() {
    return this.error;
  }

  /**
   * Get the last calculated output of the PID loop
   * @return output
   */
  public double getOutput() {
    return this.output;
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
   * Updates the PI(sans D) loop taking only the measured param. It will
   * calculated the p term, incrament the i accum, and clamp it, and return
   * the clamped output
   * @param measured
   * @return output
   */
  public double update(double measured) {
    this.error = this.setpoint - measured;

    /**
     * P output is just the error times porportional gain
     */
    this.pOutput = this.kp * this.error;

    /**
     * The iAccum should start at 0, but is incramented by error 
     * times dt. This is then clamped to the minimum/maximum of 
     * the i term. This only happens if the integral gain is set.
     */
    if(this.ki != 0) {
      this.iAccum += this.error * this.ki * this.dt; // incrament the I term by error times integral gain times delta time (numerical integration yeet)
      this.iAccum = clampIntegral(this.iAccum + this.iOutput); // clamp the term to the min/max   
    }

    /**
     * This will make sure the output of the loop does not exceed the specified min/max.
     */
    this.output = clampOutput(this.pOutput + this.iAccum); 

    return this.output;
  }
}