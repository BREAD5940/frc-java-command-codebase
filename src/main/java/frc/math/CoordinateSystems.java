package frc.math;

public class CoordinateSystems {
  // double[] xyArray = new double[2];
  public static double pi = Math.PI;

  /** 
   * Convert from polar to cartesian. Input array should be
   * in the form theta, magnitude, where theta is in degrees
   * @param polar double[theta, magnitude]
   * @return a double[] in the form x, y
   */
  public static double[] polarToCartesian(double[] polar) {
    polar[0] = Math.toRadians(polar[0]); // Convert the angle to radians because that's waht sin/cox expect
    double[] xyArray = new double[2];
    xyArray[0] = polar[1] * ( Math.cos(polar[0]) );
    xyArray[1] = polar[1] * ( Math.sin(polar[0]) );
    return xyArray;
  }

  /**
   * Return the chord length with a given angle and
   * radius of the circle. Make sure that angle is
   * within (0,180) - this function will always return
   * a positive number!
   * @param radius
   * @param angle
   * @return chord length
   */
  public static double calculateChordLen(double radius, double angle) {
    // angle = angle % 180;
    double chordLen = Math.sin(Math.toRadians(angle / 2)) * radius * 2;
    return Math.abs(chordLen);
  }

  /**
   * Return delta in gobal (x,y) and global theta given
   * old global theta, current global theta, and movement by 
   * the left and right drivetrain.
   * @param deltaLeft
   * @param deltaRight
   * @param oldAngle
   * @param currentAngle
   * @return a double[] in format (x,y) that represents delta in said axis
   */
  public static double[] calculaeDisplacement(double deltaLeft, double deltaRight, double oldAngle, double currentAngle) {
    // System.out.println("----------------------------------------");
    // System.out.println(String.format("Input: deltaLeft (deltaRight) oldAngle (currentAngle): %s (%s) %s (%s)", deltaLeft, deltaRight, oldAngle, currentAngle));

    double deltaTheta = currentAngle - oldAngle;//+ 90; 
    double chordLen;
    if (deltaLeft == deltaRight) {
      // System.out.println("Left and right distances are the same, assuming straight line movement");
      chordLen = deltaLeft;
    } else {
      double Rl = deltaLeft/ (pi * (deltaTheta/360) * 2);
      double Rr = deltaRight / (pi * (deltaTheta/360) * 2);
      double Rc = (Rl + Rr) / 2;
      chordLen = CoordinateSystems.calculateChordLen(Rc, deltaTheta);

      // System.out.println(String.format("Rl (Rr) Rc (Chordlen): %s (%s) %s (%s)", Rl, Rr, Rc, chordLen));

    }

    double theta = 180-((180-(oldAngle - currentAngle))/2) - oldAngle;

    // System.out.println("Theta should be: " + theta);

    double[] polarCoordinates = new double[2];
    polarCoordinates[0] = theta;
    polarCoordinates[1] = chordLen;

    // System.out.println(String.format("polar theta (magnitude): %s (%s)", polarCoordinates[0], polarCoordinates[1]));

    double[] cartesianCoordinates = CoordinateSystems.polarToCartesian(polarCoordinates);

    // System.out.println(String.format("deltaX (deltaY): %s (%s)", cartesianCoordinates[0], cartesianCoordinates[1] ));
    
    return cartesianCoordinates;
  }
}
