package frc.math;

public class coordinateSystems {
    // double[] xyArray = new double[2];
    
    /** 
     * Convert from polar to cartesian
     * @param double[] in the form theta, magnitude
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
    public static double chordLen(double radius, double angle) {
        // angle = angle % 180;
        double chordLen = Math.sin(Math.toRadians(angle / 2)) * radius * 2;
        return Math.abs(chordLen);
    }
}